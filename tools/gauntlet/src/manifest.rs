//! Licensed corpus manifests, deterministic sampling, and content-addressed fetching.

use std::collections::{BTreeMap, BTreeSet};
use std::fs::{self, File, OpenOptions};
use std::io::{BufReader, Read, Write};
use std::path::{Component, Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use sevenz_rust2::{ArchiveReader, Password};
use sha2::{Digest, Sha256};

use crate::GauntletError;

/// Stable manifest schema identifier.
pub const MANIFEST_SCHEMA: &str = "remus-gauntlet-manifest-v1";

/// Deterministic subset algorithm recorded in generated manifests.
pub const SAMPLE_ALGORITHM: &str = "sha256-rank-v1";

static TEMP_SEQUENCE: AtomicU64 = AtomicU64::new(0);

const DOWNLOAD_TIMEOUT: Duration = Duration::from_secs(30 * 60);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// A corpus manifest whose model bytes remain on their upstream hosts.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CorpusManifest {
    /// Schema identifier, currently [`MANIFEST_SCHEMA`].
    pub schema: String,
    /// Human-readable manifest name.
    pub name: String,
    /// Reproducible subset provenance, when this manifest is sampled.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub selection: Option<ManifestSelection>,
    /// Container archives referenced by model URL fragments.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub archives: Vec<ArchiveSource>,
    /// Models in stable display and execution order.
    pub models: Vec<ModelEntry>,
}

/// Provenance for a manifest produced by deterministic sampling.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManifestSelection {
    /// Stable algorithm name.
    pub algorithm: String,
    /// Candidate count before sampling.
    pub population: usize,
    /// Number of selected models.
    pub sample: usize,
    /// User-visible deterministic seed.
    pub seed: u64,
}

/// A downloadable archive containing one or more manifest models.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArchiveSource {
    /// Upstream archive URL, without a fragment.
    pub url: String,
    /// SHA-256 of the complete archive.
    pub sha256: String,
    /// Complete archive size in bytes.
    pub size: u64,
}

/// One externally hosted STEP model.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelEntry {
    /// Stable corpus-local identifier.
    pub id: String,
    /// Direct URL, or `URL#member=PATH` for an entry in a declared archive.
    pub url: String,
    /// SHA-256 of the uncompressed STEP model.
    pub sha256: String,
    /// Upstream license or terms class; this is disclosure, not relicensing.
    pub license_class: String,
    /// Uncompressed STEP model size in bytes.
    pub size: u64,
}

/// Local cache and runtime subset options for a fetch operation.
#[derive(Debug, Clone)]
pub struct FetchConfig {
    /// Root of the content-addressed cache.
    pub cache_dir: PathBuf,
    /// Optional runtime subset size.
    pub sample: Option<usize>,
    /// Seed used when `sample` is set.
    pub seed: u64,
    /// Exact URL-to-local-file overrides for manually acquired sources.
    pub source_files: BTreeMap<String, PathBuf>,
}

impl FetchConfig {
    /// Create an unsampled fetch configuration.
    #[must_use]
    pub fn new(cache_dir: impl Into<PathBuf>) -> Self {
        Self {
            cache_dir: cache_dir.into(),
            sample: None,
            seed: 0,
            source_files: BTreeMap::new(),
        }
    }
}

/// A fetched model and its verified content-addressed path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FetchedModel {
    /// Manifest model identifier.
    pub id: String,
    /// Verified local object path.
    pub path: PathBuf,
}

/// Settings for deriving a manifest from a pinned 7z archive.
#[derive(Debug, Clone)]
pub struct ArchiveManifestConfig {
    /// Manifest name.
    pub name: String,
    /// Prefix prepended to the archive entry's first path component.
    pub id_prefix: String,
    /// Upstream archive URL recorded in the manifest.
    pub url: String,
    /// Upstream license or terms class recorded on every model.
    pub license_class: String,
    /// Number of STEP entries to select.
    pub sample: usize,
    /// Deterministic selection seed.
    pub seed: u64,
}

/// Read and validate a manifest from disk.
///
/// # Errors
///
/// Returns an error when the file cannot be read, decoded, or validated.
pub fn read_manifest(path: &Path) -> Result<CorpusManifest, GauntletError> {
    let bytes = fs::read(path).map_err(GauntletError::io)?;
    let manifest: CorpusManifest = serde_json::from_slice(&bytes).map_err(GauntletError::json)?;
    validate_manifest(&manifest)?;
    Ok(manifest)
}

/// Write a validated manifest as stable pretty-printed JSON.
///
/// # Errors
///
/// Returns an error when validation, serialization, or the filesystem write fails.
pub fn write_manifest(path: &Path, manifest: &CorpusManifest) -> Result<(), GauntletError> {
    validate_manifest(manifest)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(GauntletError::io)?;
    }
    let mut bytes = serde_json::to_vec_pretty(manifest).map_err(GauntletError::json)?;
    bytes.push(b'\n');
    fs::write(path, bytes).map_err(GauntletError::io)
}

/// Validate schema, identity, hashes, sizes, archive references, and selection metadata.
///
/// # Errors
///
/// Returns an error for any malformed or internally inconsistent manifest field.
pub fn validate_manifest(manifest: &CorpusManifest) -> Result<(), GauntletError> {
    if manifest.schema != MANIFEST_SCHEMA {
        return Err(GauntletError::message(format!(
            "unsupported manifest schema {:?}; expected {MANIFEST_SCHEMA}",
            manifest.schema
        )));
    }
    if manifest.name.trim().is_empty() {
        return Err(GauntletError::message("manifest name must not be empty"));
    }
    if manifest.models.is_empty() {
        return Err(GauntletError::message(
            "manifest must contain at least one model",
        ));
    }
    if let Some(selection) = &manifest.selection {
        if selection.algorithm != SAMPLE_ALGORITHM {
            return Err(GauntletError::message(format!(
                "unsupported selection algorithm {:?}",
                selection.algorithm
            )));
        }
        if selection.sample != manifest.models.len() || selection.sample > selection.population {
            return Err(GauntletError::message(
                "selection sample must equal the model count and not exceed population",
            ));
        }
    }

    let mut archive_by_url: BTreeMap<&str, &ArchiveSource> = BTreeMap::new();
    for archive in &manifest.archives {
        validate_url(&archive.url, false)?;
        validate_sha256(&archive.sha256, "archive")?;
        if archive.size == 0 {
            return Err(GauntletError::message(format!(
                "archive {} has zero size",
                archive.url
            )));
        }
        if archive_by_url
            .insert(archive.url.as_str(), archive)
            .is_some()
        {
            return Err(GauntletError::message(format!(
                "duplicate archive URL {}",
                archive.url
            )));
        }
    }

    let mut ids = BTreeSet::new();
    let mut urls = BTreeSet::new();
    let mut referenced_archives = BTreeSet::new();
    for model in &manifest.models {
        if !is_safe_id(&model.id) {
            return Err(GauntletError::message(format!(
                "model id {:?} must use only ASCII letters, digits, '-', '_', or '.'",
                model.id
            )));
        }
        if !ids.insert(&model.id) {
            return Err(GauntletError::message(format!(
                "duplicate model id {}",
                model.id
            )));
        }
        if !urls.insert(&model.url) {
            return Err(GauntletError::message(format!(
                "duplicate model URL {}",
                model.url
            )));
        }
        validate_sha256(&model.sha256, &model.id)?;
        if model.license_class.trim().is_empty() {
            return Err(GauntletError::message(format!(
                "model {} has no license class",
                model.id
            )));
        }
        if model.size == 0 {
            return Err(GauntletError::message(format!(
                "model {} has zero size",
                model.id
            )));
        }
        let (base_url, member) = split_model_url(&model.url)?;
        validate_url(base_url, false)?;
        if member.is_some() {
            if !archive_by_url.contains_key(base_url) {
                return Err(GauntletError::message(format!(
                    "model {} references undeclared archive {}",
                    model.id, base_url
                )));
            }
            referenced_archives.insert(base_url);
        }
    }
    for url in archive_by_url.keys() {
        if !referenced_archives.contains(*url) {
            return Err(GauntletError::message(format!(
                "archive {url} is not referenced by any model"
            )));
        }
    }
    Ok(())
}

/// Fetch a manifest into a verified content-addressed cache.
///
/// # Errors
///
/// Returns an error when the manifest is invalid or any selected source cannot be
/// downloaded, extracted, or verified.
pub fn fetch_manifest(
    manifest_path: &Path,
    config: &FetchConfig,
) -> Result<Vec<FetchedModel>, GauntletError> {
    let manifest = read_manifest(manifest_path)?;
    fetch(&manifest, config)
}

/// Fetch selected models from a validated manifest.
///
/// # Errors
///
/// Returns an error when the manifest is invalid or any selected source cannot be
/// downloaded, extracted, or verified.
pub fn fetch(
    manifest: &CorpusManifest,
    config: &FetchConfig,
) -> Result<Vec<FetchedModel>, GauntletError> {
    validate_manifest(manifest)?;
    let selected = select_models(&manifest.models, config.sample, config.seed)?;
    fs::create_dir_all(config.cache_dir.join("objects")).map_err(GauntletError::io)?;

    let archive_by_url: BTreeMap<_, _> = manifest
        .archives
        .iter()
        .map(|archive| (archive.url.as_str(), archive))
        .collect();
    let mut archived_groups: BTreeMap<String, Vec<ModelEntry>> = BTreeMap::new();
    for model in &selected {
        let (base_url, member) = split_model_url(&model.url)?;
        if member.is_some() {
            archived_groups
                .entry(base_url.to_owned())
                .or_default()
                .push(model.clone());
        } else {
            ensure_direct_model(model, config)?;
        }
    }
    for (url, models) in archived_groups {
        let archive = archive_by_url
            .get(url.as_str())
            .ok_or_else(|| GauntletError::message(format!("archive {url} is not declared")))?;
        ensure_archive_models(archive, &models, config)?;
    }

    selected
        .into_iter()
        .map(|model| {
            let path = object_path(&config.cache_dir, &model.sha256);
            verify_cached_object(&path, model.size, &model.sha256, &model.id)?;
            Ok(FetchedModel { id: model.id, path })
        })
        .collect()
}

/// Select `sample` models by stable SHA-256 ranking, returning manifest order.
///
/// # Errors
///
/// Returns an error when the requested sample is zero or exceeds the population.
pub fn select_models(
    models: &[ModelEntry],
    sample: Option<usize>,
    seed: u64,
) -> Result<Vec<ModelEntry>, GauntletError> {
    let Some(sample) = sample else {
        return Ok(models.to_vec());
    };
    if sample == 0 {
        return Err(GauntletError::message("sample must be greater than zero"));
    }
    if sample > models.len() {
        return Err(GauntletError::message(format!(
            "sample {sample} exceeds manifest population {}",
            models.len()
        )));
    }
    let mut ranked: Vec<_> = models
        .iter()
        .enumerate()
        .map(|(index, model)| (sample_rank(seed, &model.id), model.id.as_str(), index))
        .collect();
    ranked.sort_unstable();
    let mut selected: Vec<_> = ranked
        .into_iter()
        .take(sample)
        .map(|(_, _, index)| index)
        .collect();
    selected.sort_unstable();
    Ok(selected
        .into_iter()
        .map(|index| models[index].clone())
        .collect())
}

/// Derive a reproducible STEP manifest from a pinned 7z archive without retaining models.
///
/// # Errors
///
/// Returns an error when the archive or generation settings are invalid, a selected
/// member cannot be decoded, or the resulting manifest fails validation.
#[allow(clippy::too_many_lines)]
pub fn generate_archive_manifest(
    archive_path: &Path,
    config: &ArchiveManifestConfig,
) -> Result<CorpusManifest, GauntletError> {
    if config.sample == 0 {
        return Err(GauntletError::message("sample must be greater than zero"));
    }
    if config.name.trim().is_empty()
        || config.url.trim().is_empty()
        || config.license_class.trim().is_empty()
    {
        return Err(GauntletError::message(
            "name, URL, and license class must not be empty",
        ));
    }
    validate_url(&config.url, false)?;
    let archive_metadata = fs::metadata(archive_path).map_err(GauntletError::io)?;
    let (archive_sha256, archive_size) = hash_path(archive_path)?;
    if archive_size != archive_metadata.len() {
        return Err(GauntletError::message(
            "archive size changed while it was being hashed",
        ));
    }

    let archive_file = File::open(archive_path).map_err(GauntletError::io)?;
    let mut reader = ArchiveReader::new(BufReader::new(archive_file), Password::empty())
        .map_err(|error| GauntletError::message(format!("invalid 7z archive: {error}")))?;
    let mut candidates = Vec::new();
    let mut ids = BTreeSet::new();
    for entry in &reader.archive().files {
        if entry.is_directory() || !is_step_member(entry.name()) {
            continue;
        }
        validate_member_path(entry.name())?;
        let root = entry.name().split('/').next().ok_or_else(|| {
            GauntletError::message(format!("invalid archive member {}", entry.name()))
        })?;
        let id = format!("{}{root}", config.id_prefix);
        if !ids.insert(id.clone()) {
            return Err(GauntletError::message(format!(
                "archive contains multiple STEP members for id {id}"
            )));
        }
        candidates.push((id, entry.name().to_owned(), entry.size()));
    }
    if config.sample > candidates.len() {
        return Err(GauntletError::message(format!(
            "sample {} exceeds STEP population {}",
            config.sample,
            candidates.len()
        )));
    }
    candidates.sort_by(|left, right| left.0.cmp(&right.0));
    let model_keys: Vec<_> = candidates
        .iter()
        .map(|(id, _, _)| ModelEntry {
            id: id.clone(),
            url: String::new(),
            sha256: "0".repeat(64),
            license_class: config.license_class.clone(),
            size: 1,
        })
        .collect();
    let selected_keys = select_models(&model_keys, Some(config.sample), config.seed)?;
    let selected_ids: BTreeSet<_> = selected_keys.into_iter().map(|model| model.id).collect();
    let selected: BTreeMap<_, _> = candidates
        .into_iter()
        .filter(|(id, _, _)| selected_ids.contains(id))
        .map(|(id, member, size)| (member, (id, size)))
        .collect();

    let mut hashes: BTreeMap<String, (String, u64)> = BTreeMap::new();
    reader
        .for_each_entries(|entry, entry_reader| {
            let result = if let Some((id, expected_size)) = selected.get(entry.name()) {
                if entry.size() == *expected_size {
                    hash_reader(entry_reader, *expected_size).map(|value| {
                        hashes.insert(id.clone(), value);
                    })
                } else {
                    Err(GauntletError::message(format!(
                        "archive member {} changed size",
                        entry.name()
                    )))
                }
            } else {
                std::io::copy(entry_reader, &mut std::io::sink())
                    .map(|_| ())
                    .map_err(GauntletError::io)
            };
            result
                .map(|()| true)
                .map_err(|error| sevenz_rust2::Error::Other(error.to_string().into()))
        })
        .map_err(|error| GauntletError::message(format!("7z extraction failed: {error}")))?;
    if hashes.len() != selected.len() {
        return Err(GauntletError::message(format!(
            "hashed {} selected members, expected {}",
            hashes.len(),
            selected.len()
        )));
    }

    let mut models = Vec::with_capacity(selected.len());
    for (member, (id, expected_size)) in selected {
        let (sha256, size) = hashes.remove(&id).ok_or_else(|| {
            GauntletError::message(format!("selected archive member {member} was not read"))
        })?;
        if size != expected_size {
            return Err(GauntletError::message(format!(
                "archive member {member} size mismatch"
            )));
        }
        models.push(ModelEntry {
            id,
            url: format!("{}#member={member}", config.url),
            sha256,
            license_class: config.license_class.clone(),
            size,
        });
    }
    models.sort_by(|left, right| left.id.cmp(&right.id));
    let manifest = CorpusManifest {
        schema: MANIFEST_SCHEMA.to_owned(),
        name: config.name.clone(),
        selection: Some(ManifestSelection {
            algorithm: SAMPLE_ALGORITHM.to_owned(),
            population: reader
                .archive()
                .files
                .iter()
                .filter(|entry| !entry.is_directory() && is_step_member(entry.name()))
                .count(),
            sample: config.sample,
            seed: config.seed,
        }),
        archives: vec![ArchiveSource {
            url: config.url.clone(),
            sha256: archive_sha256,
            size: archive_size,
        }],
        models,
    };
    validate_manifest(&manifest)?;
    Ok(manifest)
}

fn ensure_direct_model(model: &ModelEntry, config: &FetchConfig) -> Result<(), GauntletError> {
    let path = object_path(&config.cache_dir, &model.sha256);
    if path.is_file() {
        return verify_cached_object(&path, model.size, &model.sha256, &model.id);
    }
    let source = source_reader(&model.url, config)?;
    store_reader(
        source,
        &config.cache_dir,
        model.size,
        &model.sha256,
        &model.id,
    )?;
    Ok(())
}

fn ensure_archive_models(
    archive: &ArchiveSource,
    models: &[ModelEntry],
    config: &FetchConfig,
) -> Result<(), GauntletError> {
    let mut missing = BTreeMap::new();
    for model in models {
        let path = object_path(&config.cache_dir, &model.sha256);
        if path.is_file() {
            verify_cached_object(&path, model.size, &model.sha256, &model.id)?;
            continue;
        }
        let (_, member) = split_model_url(&model.url)?;
        let member = member.ok_or_else(|| {
            GauntletError::message(format!("model {} has no archive member", model.id))
        })?;
        missing.insert(member.to_owned(), model.clone());
    }
    if missing.is_empty() {
        return Ok(());
    }

    let cached_archive_path = object_path(&config.cache_dir, &archive.sha256);
    let archive_path = if let Some(source_path) = config.source_files.get(&archive.url) {
        verify_cached_object(source_path, archive.size, &archive.sha256, &archive.url)?;
        source_path.clone()
    } else if cached_archive_path.is_file() {
        verify_cached_object(
            &cached_archive_path,
            archive.size,
            &archive.sha256,
            &archive.url,
        )?;
        cached_archive_path
    } else {
        let source = source_reader(&archive.url, config)?;
        store_reader(
            source,
            &config.cache_dir,
            archive.size,
            &archive.sha256,
            &archive.url,
        )?
    };

    let mut found = BTreeSet::new();
    sevenz_rust2::decompress_file_with_extract_fn(
        &archive_path,
        config.cache_dir.join("tmp"),
        |entry, entry_reader, _destination| {
            let result = if let Some(model) = missing.get(entry.name()) {
                store_reader(
                    entry_reader,
                    &config.cache_dir,
                    model.size,
                    &model.sha256,
                    &model.id,
                )
                .map(|_| {
                    found.insert(entry.name().to_owned());
                })
            } else {
                std::io::copy(entry_reader, &mut std::io::sink())
                    .map(|_| ())
                    .map_err(GauntletError::io)
            };
            result
                .map(|()| found.len() != missing.len())
                .map_err(|error| sevenz_rust2::Error::Other(error.to_string().into()))
        },
    )
    .map_err(|error| GauntletError::message(format!("7z extraction failed: {error}")))?;

    let not_found: Vec<_> = missing
        .keys()
        .filter(|member| !found.contains(*member))
        .cloned()
        .collect();
    if !not_found.is_empty() {
        return Err(GauntletError::message(format!(
            "archive {} is missing manifest members: {}",
            archive.url,
            not_found.join(", ")
        )));
    }
    Ok(())
}

fn source_reader(url: &str, config: &FetchConfig) -> Result<Box<dyn Read>, GauntletError> {
    if let Some(path) = config.source_files.get(url) {
        return File::open(path)
            .map(|file| Box::new(BufReader::new(file)) as Box<dyn Read>)
            .map_err(GauntletError::io);
    }
    if let Some(path) = url.strip_prefix("file://") {
        return File::open(path)
            .map(|file| Box::new(BufReader::new(file)) as Box<dyn Read>)
            .map_err(GauntletError::io);
    }
    if !url.starts_with("https://") && !url.starts_with("http://") {
        return Err(GauntletError::message(format!(
            "no local source override for unsupported URL {url}"
        )));
    }
    let agent = ureq::Agent::config_builder()
        .timeout_global(Some(DOWNLOAD_TIMEOUT))
        .timeout_connect(Some(CONNECT_TIMEOUT))
        .build()
        .new_agent();
    let response = agent
        .get(url)
        .call()
        .map_err(|error| GauntletError::message(format!("download {url} failed: {error}")))?;
    let (_, body) = response.into_parts();
    Ok(Box::new(body.into_reader()))
}

fn store_reader(
    mut reader: impl Read,
    cache_dir: &Path,
    expected_size: u64,
    expected_sha256: &str,
    label: &str,
) -> Result<PathBuf, GauntletError> {
    let object = object_path(cache_dir, expected_sha256);
    if object.is_file() {
        verify_cached_object(&object, expected_size, expected_sha256, label)?;
        return Ok(object);
    }
    let (temporary, mut output) = create_temporary(cache_dir)?;
    let result = copy_hash(&mut reader, Some(&mut output), expected_size).and_then(
        |(actual_sha256, actual_size)| {
            if actual_size != expected_size {
                return Err(GauntletError::message(format!(
                    "{label} size mismatch: expected {expected_size}, got {actual_size}"
                )));
            }
            if actual_sha256 != expected_sha256 {
                return Err(GauntletError::message(format!(
                    "{label} SHA-256 mismatch: expected {expected_sha256}, got {actual_sha256}"
                )));
            }
            output.flush().map_err(GauntletError::io)
        },
    );
    drop(output);
    if let Err(error) = result {
        let _ = fs::remove_file(&temporary);
        return Err(error);
    }
    if let Some(parent) = object.parent() {
        fs::create_dir_all(parent).map_err(GauntletError::io)?;
    }
    if object.is_file() {
        verify_cached_object(&object, expected_size, expected_sha256, label)?;
        fs::remove_file(&temporary).map_err(GauntletError::io)?;
    } else {
        fs::rename(&temporary, &object).map_err(GauntletError::io)?;
    }
    Ok(object)
}

fn create_temporary(cache_dir: &Path) -> Result<(PathBuf, File), GauntletError> {
    let directory = cache_dir.join("tmp");
    fs::create_dir_all(&directory).map_err(GauntletError::io)?;
    for _ in 0..1000 {
        let sequence = TEMP_SEQUENCE.fetch_add(1, Ordering::Relaxed);
        let path = directory.join(format!("{}-{sequence}.part", std::process::id()));
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(file) => return Ok((path, file)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(error) => return Err(GauntletError::io(error)),
        }
    }
    Err(GauntletError::message(
        "could not allocate a unique cache temporary file",
    ))
}

fn verify_cached_object(
    path: &Path,
    expected_size: u64,
    expected_sha256: &str,
    label: &str,
) -> Result<(), GauntletError> {
    let (actual_sha256, actual_size) = hash_path(path)?;
    if actual_size != expected_size || actual_sha256 != expected_sha256 {
        return Err(GauntletError::message(format!(
            "cached object for {label} failed verification; remove {} and retry",
            path.display()
        )));
    }
    Ok(())
}

fn hash_path(path: &Path) -> Result<(String, u64), GauntletError> {
    let file = File::open(path).map_err(GauntletError::io)?;
    hash_reader(BufReader::new(file), u64::MAX)
}

fn hash_reader(mut reader: impl Read, expected_max: u64) -> Result<(String, u64), GauntletError> {
    copy_hash(&mut reader, None::<&mut File>, expected_max)
}

fn copy_hash<W: Write>(
    reader: &mut dyn Read,
    mut writer: Option<&mut W>,
    expected_max: u64,
) -> Result<(String, u64), GauntletError> {
    let mut hasher = Sha256::new();
    let mut size = 0_u64;
    let mut buffer = vec![0_u8; 64 * 1024].into_boxed_slice();
    loop {
        let count = reader.read(&mut buffer).map_err(GauntletError::io)?;
        if count == 0 {
            break;
        }
        size =
            size.checked_add(u64::try_from(count).map_err(|_| {
                GauntletError::message("reader produced an unsupported chunk size")
            })?)
            .ok_or_else(|| GauntletError::message("source size overflow"))?;
        if size > expected_max {
            return Err(GauntletError::message(format!(
                "source exceeds declared size {expected_max}"
            )));
        }
        hasher.update(&buffer[..count]);
        if let Some(output) = writer.as_deref_mut() {
            output
                .write_all(&buffer[..count])
                .map_err(GauntletError::io)?;
        }
    }
    Ok((hex_digest(hasher.finalize().as_slice()), size))
}

fn object_path(cache_dir: &Path, sha256: &str) -> PathBuf {
    cache_dir
        .join("objects")
        .join(&sha256[..2])
        .join(&sha256[2..])
}

fn sample_rank(seed: u64, id: &str) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(seed.to_le_bytes());
    hasher.update([0]);
    hasher.update(id.as_bytes());
    hasher.finalize().into()
}

fn split_model_url(url: &str) -> Result<(&str, Option<&str>), GauntletError> {
    let Some((base, fragment)) = url.split_once('#') else {
        return Ok((url, None));
    };
    if base.contains('#') || fragment.contains('#') {
        return Err(GauntletError::message(format!(
            "model URL has multiple fragments: {url}"
        )));
    }
    let member = fragment.strip_prefix("member=").ok_or_else(|| {
        GauntletError::message(format!("unsupported model URL fragment in {url}"))
    })?;
    validate_member_path(member)?;
    Ok((base, Some(member)))
}

fn validate_url(url: &str, allow_fragment: bool) -> Result<(), GauntletError> {
    if url.trim().is_empty() || (!allow_fragment && url.contains('#')) {
        return Err(GauntletError::message(format!(
            "invalid source URL {url:?}"
        )));
    }
    if !url.starts_with("https://") && !url.starts_with("http://") && !url.starts_with("file://") {
        return Err(GauntletError::message(format!(
            "source URL must use https, http, or file: {url}"
        )));
    }
    Ok(())
}

fn validate_member_path(member: &str) -> Result<(), GauntletError> {
    if member.is_empty() || member.contains('\\') || member.contains('#') {
        return Err(GauntletError::message(format!(
            "unsafe archive member path {member:?}"
        )));
    }
    if Path::new(member)
        .components()
        .any(|component| !matches!(component, Component::Normal(_)))
    {
        return Err(GauntletError::message(format!(
            "unsafe archive member path {member:?}"
        )));
    }
    Ok(())
}

fn validate_sha256(value: &str, label: &str) -> Result<(), GauntletError> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(GauntletError::message(format!(
            "{label} SHA-256 must be 64 lowercase hexadecimal characters"
        )));
    }
    Ok(())
}

fn is_safe_id(value: &str) -> bool {
    !value.is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
}

fn is_step_member(name: &str) -> bool {
    Path::new(name)
        .extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| {
            extension.eq_ignore_ascii_case("step") || extension.eq_ignore_ascii_case("stp")
        })
}

fn hex_digest(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for &byte in bytes {
        output.push(char::from(HEX[usize::from(byte >> 4)]));
        output.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    output
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!(
            "remus-gauntlet-manifest-{}-{nanos}-{name}",
            std::process::id()
        ))
    }

    fn model(id: &str, source: &Path, contents: &[u8]) -> ModelEntry {
        let (sha256, size) = hash_reader(contents, u64::MAX).unwrap();
        ModelEntry {
            id: id.to_owned(),
            url: format!("file://{}", source.display()),
            sha256,
            license_class: "test-only".to_owned(),
            size,
        }
    }

    fn manifest(models: Vec<ModelEntry>) -> CorpusManifest {
        CorpusManifest {
            schema: MANIFEST_SCHEMA.to_owned(),
            name: "test".to_owned(),
            selection: None,
            archives: Vec::new(),
            models,
        }
    }

    #[test]
    fn deterministic_sampling_is_seeded_and_order_preserving() {
        let root = temp_dir("sample");
        let models: Vec<_> = (0..20)
            .map(|index| model(&format!("m{index:02}"), &root, b"x"))
            .collect();
        let first = select_models(&models, Some(7), 42).unwrap();
        let repeat = select_models(&models, Some(7), 42).unwrap();
        let other = select_models(&models, Some(7), 43).unwrap();
        assert_eq!(first, repeat);
        assert_ne!(first, other);
        let indices: Vec<_> = first
            .iter()
            .map(|entry| models.iter().position(|model| model == entry).unwrap())
            .collect();
        assert!(indices.windows(2).all(|pair| pair[0] < pair[1]));
    }

    #[test]
    fn direct_fetch_populates_and_reuses_verified_content_cache() {
        let root = temp_dir("direct");
        fs::create_dir_all(&root).unwrap();
        let source = root.join("source.step");
        fs::write(&source, b"ISO-10303-21;END-ISO-10303-21;").unwrap();
        let entry = model("direct", &source, &fs::read(&source).unwrap());
        let config = FetchConfig::new(root.join("cache"));
        let first = fetch(&manifest(vec![entry.clone()]), &config).unwrap();
        fs::remove_file(&source).unwrap();
        let second = fetch(&manifest(vec![entry]), &config).unwrap();
        assert_eq!(first, second);
        assert_eq!(
            fs::read(&first[0].path).unwrap(),
            b"ISO-10303-21;END-ISO-10303-21;"
        );
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn hash_mismatch_refuses_without_caching_bytes() {
        let root = temp_dir("bad-hash");
        fs::create_dir_all(&root).unwrap();
        let source = root.join("source.step");
        fs::write(&source, b"actual").unwrap();
        let mut entry = model("bad", &source, b"expected");
        entry.size = 6;
        let config = FetchConfig::new(root.join("cache"));
        let error = fetch(&manifest(vec![entry.clone()]), &config).unwrap_err();
        assert!(error.to_string().contains("SHA-256 mismatch"));
        assert!(!object_path(&config.cache_dir, &entry.sha256).exists());
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn archive_generator_and_fetcher_verify_members_both_sides() {
        let root = temp_dir("archive");
        let source_root = root.join("source");
        fs::create_dir_all(source_root.join("00000000")).unwrap();
        fs::create_dir_all(source_root.join("00000001")).unwrap();
        fs::write(
            source_root.join("00000000/first.step"),
            b"ISO-10303-21;first;END-ISO-10303-21;",
        )
        .unwrap();
        fs::write(
            source_root.join("00000001/second.step"),
            b"ISO-10303-21;second;END-ISO-10303-21;",
        )
        .unwrap();
        let archive_path = root.join("models.7z");
        sevenz_rust2::compress_to_path(&source_root, &archive_path).unwrap();
        let source_url = "https://example.invalid/models.7z";
        let generated = generate_archive_manifest(
            &archive_path,
            &ArchiveManifestConfig {
                name: "archive-test".to_owned(),
                id_prefix: "abc-".to_owned(),
                url: source_url.to_owned(),
                license_class: "test-only".to_owned(),
                sample: 2,
                seed: 9,
            },
        )
        .unwrap();
        assert_eq!(generated.models.len(), 2);

        let mut config = FetchConfig::new(root.join("cache"));
        config
            .source_files
            .insert(source_url.to_owned(), archive_path.clone());
        let fetched = fetch(&generated, &config).unwrap();
        assert_eq!(fetched.len(), 2);
        assert!(!object_path(&config.cache_dir, &generated.archives[0].sha256).exists());

        let mut missing = generated;
        missing.models[0].url = format!("{source_url}#member=missing/model.step");
        let missing_cache = root.join("missing-cache");
        let mut missing_config = FetchConfig::new(&missing_cache);
        missing_config
            .source_files
            .insert(source_url.to_owned(), archive_path);
        let error = fetch(&missing, &missing_config).unwrap_err();
        assert!(error.to_string().contains("missing manifest members"));
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn unsafe_archive_member_is_refused() {
        let mut value = manifest(vec![ModelEntry {
            id: "unsafe".to_owned(),
            url: "https://example.invalid/models.7z#member=../escape.step".to_owned(),
            sha256: "0".repeat(64),
            license_class: "test-only".to_owned(),
            size: 1,
        }]);
        value.archives.push(ArchiveSource {
            url: "https://example.invalid/models.7z".to_owned(),
            sha256: "1".repeat(64),
            size: 1,
        });
        let error = validate_manifest(&value).unwrap_err();
        assert!(error.to_string().contains("unsafe archive member"));
    }

    #[test]
    fn unsafe_model_id_is_refused() {
        let value = manifest(vec![ModelEntry {
            id: "unsafe\trow".to_owned(),
            url: "https://example.invalid/model.step".to_owned(),
            sha256: "0".repeat(64),
            license_class: "test-only".to_owned(),
            size: 1,
        }]);
        let error = validate_manifest(&value).unwrap_err();
        assert!(error.to_string().contains("model id"));
    }

    #[test]
    fn committed_manifests_validate_and_smoke_matches_recorded_sample() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("manifests");
        let abc = read_manifest(&directory.join("abc-1k.json")).unwrap();
        let mambo = read_manifest(&directory.join("mambo.json")).unwrap();
        let smoke = read_manifest(&directory.join("smoke.json")).unwrap();

        assert_eq!(abc.models.len(), 1_000);
        assert_eq!(
            abc.selection,
            Some(ManifestSelection {
                algorithm: SAMPLE_ALGORITHM.to_owned(),
                population: 10_000,
                sample: 1_000,
                seed: 20_260_831,
            })
        );
        assert_eq!(
            abc.archives,
            vec![ArchiveSource {
                url: "https://archive.nyu.edu/bitstream/2451/44309/3/abc_0000_step_v00.7z"
                    .to_owned(),
                sha256: "52e6dd1b6fa38e3cd99af59b662370829129540030975919c3e256dce6ad1dbe"
                    .to_owned(),
                size: 1_594_129_754,
            }]
        );
        assert_eq!(mambo.models.len(), 113);
        assert_eq!(smoke.models.len(), 50);
        assert_eq!(
            smoke.models,
            select_models(&mambo.models, Some(50), 20_260_831).unwrap()
        );
    }
}
