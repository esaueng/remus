import os
import subprocess
import tempfile
import unittest

from check_public_commit_metadata import read_commits, validate_commits, validate_identity


class PublicCommitMetadataTests(unittest.TestCase):
    def test_accepts_github_noreply_user_and_bot_addresses(self):
        self.assertEqual(validate_identity("Ada Lovelace", "123+ada@users.noreply.github.com"), [])
        self.assertEqual(
            validate_identity("GitHub Actions[bot]", "41898282+github-actions[bot]@users.noreply.github.com"),
            [],
        )
        self.assertEqual(validate_identity("GitHub", "noreply@github.com"), [])

    def test_rejects_unofficial_noreply_domains_without_disclosing_identities(self):
        for email in ("example@noreply.github.com", "example@users.noreply.github.com.example.invalid"):
            findings = validate_commits([{
                "hash": "0123456789abcdef",
                "author_name": "Example Person", "author_email": email,
                "committer_name": "GitHub", "committer_email": "noreply@github.com",
            }])
            self.assertEqual(len(findings), 1)
            self.assertIn(": author email must use ", findings[0])
            self.assertNotIn(email, "\n".join(findings))

    def test_accepts_claude_public_automation_identities(self):
        for name in ("Claude", "Claude Code", "CLAUDE"):
            self.assertEqual(validate_identity(name, "noreply@anthropic.com"), [])
        self.assertEqual(validate_identity("Claude", "NOREPLY@ANTHROPIC.COM"), [])
        for name, email in (
            ("claude[bot]", "209825114+claude[bot]@users.noreply.github.com"),
            ("Claude Code", "208546643+claude-code-action[bot]@users.noreply.github.com"),
            ("dependabot[bot]", "49699333+dependabot[bot]@users.noreply.github.com"),
            ("renovate[bot]", "29139614+renovate[bot]@users.noreply.github.com"),
        ):
            self.assertEqual(validate_identity(name, email), [])
        self.assertEqual(validate_commits([{
            "hash": "0123456789abcdef",
            "author_name": "Claude", "author_email": "noreply@anthropic.com",
            "committer_name": "Claude Code", "committer_email": "noreply@anthropic.com",
        }]), [])

    def test_bot_exception_preserves_personal_identity_checks(self):
        for name, email in (
            ("Example Person", "noreply@anthropic.com"),
            ("Claude", "example@anthropic.com"),
            ("Claude", "noreply@anthropic.com.example.invalid"),
            ("Claude", "noreply+example@anthropic.com"),
            ("Claude", "noreply@example.invalid"),
        ):
            self.assertTrue(validate_identity(name, email))
        self.assertTrue(validate_identity(" Claude ", "noreply@anthropic.com"))
        self.assertTrue(validate_identity("Claude\u0000", "noreply@anthropic.com"))
        findings = validate_commits([{
            "hash": "0123456789abcdef",
            "author_name": "Claude", "author_email": "noreply@anthropic.com",
            "committer_name": "Example Person", "committer_email": "private@example.invalid",
        }])
        self.assertEqual(len(findings), 1)
        self.assertIn(": committer email must use ", findings[0])
        self.assertNotIn("private@example.invalid", "\n".join(findings))

    def test_rejects_malformed_public_and_placeholder_identities(self):
        self.assertGreaterEqual(len(validate_identity("Your Name", "ada@example.com")), 2)
        self.assertTrue(any("placeholder" in problem for problem in validate_identity("runner", "runner@users.noreply.github.com")))
        self.assertTrue(any("control" in problem for problem in validate_identity("Ada\nLovelace", "ada@users.noreply.github.com")))
        self.assertTrue(any("GitHub noreply" in problem for problem in validate_identity("Ada Lovelace", "ada@gmail.com")))
        findings = validate_commits(
            [
                {
                    "hash": "0123456789abcdef",
                    "author_name": "Ada Lovelace",
                    "author_email": "private@example.com",
                    "committer_name": "Ada Lovelace",
                    "committer_email": "committer@example.net",
                }
            ]
        )
        self.assertEqual(len(findings), 2)
        self.assertEqual(findings[0], "0123456789ab: author email must use a GitHub noreply domain or an approved public bot identity")
        self.assertEqual(findings[1], "0123456789ab: committer email must use a GitHub noreply domain or an approved public bot identity")
        self.assertNotIn("private@example.com", "\n".join(findings))

    def commit(self, message):
        return {
            "hash": "0123456789abcdef",
            "author_name": "Ada Lovelace", "author_email": "123+ada@users.noreply.github.com",
            "committer_name": "GitHub", "committer_email": "noreply@github.com",
            "message": message,
        }

    def test_accepts_public_identity_trailers(self):
        message = (
            "feat: subject\n\nBody.\n\n"
            "Co-authored-by: Ada Lovelace <123+ada@users.noreply.github.com>\n"
            "co-authored-by: Claude <noreply@anthropic.com>\n"
            "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>\n"
            "Co-authored-by: github-actions[bot] <41898282+github-actions[bot]@users.noreply.github.com>\n"
            "Signed-off-by: Ada Lovelace <123+ada@users.noreply.github.com>\n"
            "Signed-off-by: dependabot[bot] <support@github.com>\n"
        )
        self.assertEqual(validate_commits([self.commit(message)]), [])
        # A message without trailers, or a commit read without one, still passes.
        self.assertEqual(validate_commits([self.commit("fix: no trailers")]), [])
        commit = self.commit("")
        del commit["message"]
        self.assertEqual(validate_commits([commit]), [])

    def test_rejects_private_trailer_identities_without_disclosing_them(self):
        for trailer, problem in (
            ("Co-authored-by: Ada Lovelace <private@example.com>", "co-authored-by trailer email must use "),
            ("Signed-off-by: Ada Lovelace <private@example.com>", "signed-off-by trailer email must use "),
            ("CO-AUTHORED-BY:Ada Lovelace <private@example.com>", "co-authored-by trailer email must use "),
            ("  Co-authored-by: Ada Lovelace <private@example.com>", "co-authored-by trailer email must use "),
            ("Co-authored-by: Example Person <noreply@anthropic.com>", "co-authored-by trailer email must use "),
            ("Co-authored-by: Claude Opus 5.5 extra <noreply@anthropic.com>", "co-authored-by trailer email must use "),
            ("Signed-off-by: Example Person <support@github.com>", "signed-off-by trailer email must use "),
            ("Co-authored-by: Your Name <123+ada@users.noreply.github.com>", "co-authored-by trailer name is an obvious placeholder"),
            ("Co-authored-by: Ada Lovelace private@example.com", "co-authored-by trailer is not of the form"),
            ("Signed-off-by: <123+ada@users.noreply.github.com>", "signed-off-by trailer name is blank"),
        ):
            with self.subTest(trailer=trailer):
                # A trailer outside the final paragraph is still checked.
                findings = validate_commits([self.commit(f"feat: subject\n\n{trailer}\n\nMore body.")])
                self.assertEqual(len(findings), 1, findings)
                self.assertTrue(findings[0].startswith("0123456789ab: " + problem), findings[0])
                self.assertNotIn("private@example.com", "\n".join(findings))

    def test_reports_each_bad_trailer_alongside_author_findings(self):
        commit = self.commit(
            "feat: subject\n\n"
            "Co-authored-by: One <one@example.com>\n"
            "Signed-off-by: Two <two@example.com>\n"
        )
        commit["author_email"] = "author@example.com"
        findings = validate_commits([commit])
        self.assertEqual([finding.split(": ", 1)[1].split(" ", 2)[:2] for finding in findings],
                         [["author", "email"], ["co-authored-by", "trailer"], ["signed-off-by", "trailer"]])

    def test_reads_messages_from_git(self):
        with tempfile.TemporaryDirectory() as directory:
            def git(*args):
                return subprocess.run(["git", "-C", directory, *args], check=True,
                                      capture_output=True, text=True).stdout.strip()

            env = dict(os.environ, GIT_AUTHOR_NAME="Ada Lovelace",
                       GIT_AUTHOR_EMAIL="123+ada@users.noreply.github.com",
                       GIT_COMMITTER_NAME="Ada Lovelace",
                       GIT_COMMITTER_EMAIL="123+ada@users.noreply.github.com")
            git("init", "-q")
            # commit-tree writes commits without running any hook.
            tree = git("write-tree")
            base = subprocess.run(["git", "-C", directory, "commit-tree", tree, "-m", "chore: base"],
                                  check=True, env=env, capture_output=True, text=True).stdout.strip()
            head = subprocess.run(["git", "-C", directory, "commit-tree", tree, "-p", base, "-m",
                                   "feat: change\n\nCo-authored-by: One <one@example.com>"],
                                  check=True, env=env, capture_output=True, text=True).stdout.strip()
            cwd = os.getcwd()
            os.chdir(directory)
            try:
                commits = read_commits(base, head)
            finally:
                os.chdir(cwd)
        self.assertEqual(len(commits), 1)
        self.assertIn("Co-authored-by: One <one@example.com>", commits[0]["message"])
        self.assertEqual(len(validate_commits(commits)), 1)


if __name__ == "__main__":
    unittest.main()
