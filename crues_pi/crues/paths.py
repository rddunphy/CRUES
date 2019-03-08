import os


def config_directory():
    dirname = os.path.dirname(__file__)
    root = os.path.dirname(dirname)
    return os.path.join(root, "config")


def config_file(file_name):
    return os.path.join(config_directory(), file_name)
