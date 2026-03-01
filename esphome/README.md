# ESPHome Local Quick Start

This is a lightweight reminder for running ESPHome locally with a virtualenv.

## One‑time Setup

From your cloned ESPHome repo:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -U pip
```

Install ESPHome (choose one):

- From PyPI:
  ```bash
  pip install esphome
  ```
- From your local clone:
  ```bash
  pip install -e .
  ```

## Daily Usage

Activate the venv:

```bash
source /path/to/esphome-repo/venv/bin/activate
```

Then from your config directory:

```bash
esphome compile path/to/config.yaml
esphome run path/to/config.yaml
esphome logs path/to/config.yaml
```

## Notes

- `esphome run` compiles, uploads, and starts logs in one step.
- `esphome compile` is useful for verifying configs without flashing.
