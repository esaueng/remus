"""Account renames preserve provenance only for the pinned account ID."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("provenance", Path(__file__).with_name("check-apache-replay-provenance.py"))
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class ProvenanceIdentityTests(unittest.TestCase):
    def test_renamed_account_preserves_identity(self):
        current = ("Peter", "171875562+petertoday@users.noreply.github.com")
        self.assertEqual(policy.normalized_replay_author("Peter", "171875562+example-alias@users.noreply.github.com"), current)
        self.assertIn(current, policy.EXPECTED_ALLOWED_AUTHORS)

    def test_other_accounts_and_addresses_remain_unapproved(self):
        for name, email in (
            ("Peter", "123+example-alias@users.noreply.github.com"),
            ("Peter", "171875562+example-alias@users.noreply.github.com.example.invalid"),
            ("Example Person", "171875562+example-alias@users.noreply.github.com"),
            ("Peter", "person@example.invalid"),
        ):
            self.assertNotIn(policy.normalized_replay_author(name, email), policy.EXPECTED_ALLOWED_AUTHORS)


if __name__ == "__main__":
    unittest.main()
