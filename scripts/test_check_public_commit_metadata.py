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


if __name__ == "__main__":
    unittest.main()
