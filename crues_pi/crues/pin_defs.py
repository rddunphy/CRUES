"""Pin definitions for Raspberry Pi GPIO."""

import json
import os


def _get_pins_config_path():
    dirname = os.path.dirname(__file__)
    return os.path.join(dirname, "../config/pins_v1.json")


class Pins:
    def __init__(self, path=None):
        if not path:
            path = _get_pins_config_path()
        with open(path) as f:
            self._load(f)

    def _load(self, path):
        d = json.load(path)
        self.UCT = d["uct"]
        self.UCE = d["uce"]
        self.ULT = d["ult"]
        self.ULE = d["ule"]
        self.URT = d["urt"]
        self.URE = d["ure"]
        self.SDA = d["sda"]
        self.SCL = d["scl"]
        self.MLD = d["mld"]
        self.MRD = d["mrd"]
        self.MLP = d["mlp"]
        self.MRP = d["mrp"]
        self.MS = d["ms"]
        self.ELA = d["ela"]
        self.ERA = d["era"]
        self.ELB = d["elb"]
        self.ERB = d["erb"]
        self.LG = d["lg"]
        self.LR = d["lr"]
