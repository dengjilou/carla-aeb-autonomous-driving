import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scenarios'))
from report_independent_aeb import configuration_sha256


class IndependentManifestTests(unittest.TestCase):
    def test_both_checkout_line_endings_match_the_frozen_digest(self):
        raw = (ROOT/'configs/independent_aeb.json').read_bytes().replace(b'\r\n',b'\n')
        expected = json.loads((ROOT/'docs/independent_aeb_manifest.json').read_text())['config_lf_sha256']
        self.assertEqual(configuration_sha256(raw),expected)
        self.assertEqual(configuration_sha256(raw.replace(b'\n',b'\r\n')),expected)
        self.assertNotEqual(configuration_sha256(raw.replace(b'"seed": 42',b'"seed": 43')),expected)
