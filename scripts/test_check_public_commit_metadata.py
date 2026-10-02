import unittest

from check_public_commit_metadata import validate_commits, validate_identity


class PublicCommitMetadataTests(unittest.TestCase):
    def test_accepts_github_noreply_user_and_bot_addresses(self):
        self.assertEqual(validate_identity("Ada Lovelace", "123+ada@users.noreply.github.com"), [])
        self.assertEqual(
            validate_identity("GitHub Actions[bot]", "41898282+github-actions[bot]@users.noreply.github.com"),
            [],
        )
        self.assertEqual(validate_identity("Legacy account", "legacy-user@noreply.github.com"), [])
        self.assertEqual(validate_identity("GitHub", "noreply@github.com"), [])

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
        self.assertEqual(findings[0], "0123456789ab: author email must use a GitHub noreply domain")
        self.assertEqual(findings[1], "0123456789ab: committer email must use a GitHub noreply domain")
        self.assertNotIn("private@example.com", "\n".join(findings))


if __name__ == "__main__":
    unittest.main()
