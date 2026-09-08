import os
from pathlib import Path
import textwrap
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / '.github/workflows/select-runner.yml'
inline = SOURCE.read_text().split("python3 - <<'PY'\n", 1)[1].rsplit('          PY', 1)[0]
namespace = {'__name__': 'fleet_selector_test'}
exec(compile(textwrap.dedent(inline), str(SOURCE), 'exec'), namespace)


class FleetSelectorTests(unittest.TestCase):
    def test_ordered_targets_and_expiry_validation(self):
        valid = namespace['valid_target']
        for target in ('ci-server-jane', 'ci-server-john', 'github-hosted'):
            response = dict(target=target, policy='unavailable-only', generated_at=1000, expires_at=1030)
            self.assertEqual(valid(response, 1001), target)
            self.assertEqual(valid(response, 1030), 'github-hosted')
            self.assertEqual(valid(response, 900), 'github-hosted')
        for invalid in (None, [], {}, {'target': ['ci-server-jane']}, {'target': 'untrusted'}):
            self.assertEqual(valid(invalid, 1001), 'github-hosted')

    def test_disabled_retains_existing_routing_without_request(self):
        def forbidden(*args):
            self.fail('disabled routing contacted endpoint')
        self.assertEqual(namespace['choose'](False, forbidden), 'ci-small')

    @patch.dict(os.environ, {'ACTIONS_ID_TOKEN_REQUEST_URL': 'https://example.test/token?x=1',
                             'ACTIONS_ID_TOKEN_REQUEST_TOKEN': 'synthetic-request-token'})
    def test_oidc_audience_and_static_endpoint(self):
        calls = []
        def request(url, token, method='GET'):
            calls.append((url, token, method))
            if len(calls) == 1:
                return {'value': 'synthetic-oidc-token'}
            return dict(target='ci-server-jane', policy='unavailable-only', generated_at=1000, expires_at=1030)
        self.assertEqual(namespace['choose'](True, request, lambda: 1001), 'ci-server-jane')
        self.assertIn('&audience=https%3A%2F%2Fci.esau.app%2Frouting%2Fv1%2Ftarget', calls[0][0])
        self.assertEqual(calls[1], (namespace['ENDPOINT'], 'synthetic-oidc-token', 'POST'))

    def test_service_failure_falls_back_without_retrying_workload(self):
        def failed(*args):
            raise OSError('synthetic network failure')
        self.assertEqual(namespace['choose'](True, failed), 'github-hosted')


if __name__ == '__main__':
    unittest.main()
