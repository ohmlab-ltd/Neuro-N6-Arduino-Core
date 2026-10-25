# Security policy

## Reporting a vulnerability

Email security@ohmlab.co.uk with a description, the affected versions and
steps to reproduce. Do not open a public issue.

A report is acknowledged within 5 working days, with a status update at least
once a month until it is resolved.

## Support

Security updates are provided until 31 December 2031, the support period
published at https://neuron6.com/security. Releases are announced on GitHub
and in the Boards Manager package index.

## Scope

The Arduino core, its bundled libraries, and the bootloader image attached to
its releases.

## Limits

- Firmware images are checked by CRC, which detects corruption and not
  tampering. They are not signed.
- There is no network update path. Updates are installed over USB.
- Model weights in external flash are not encrypted or authenticated.
- The USB serial port and the vendor endpoint are not authenticated. A host on
  either can reboot the board into its bootloader, and the vendor endpoint
  gives it the password of the access point the board hosts.
- WiFi takes WPA2 and WPA3 only. The video link to a host runs on the board's
  own access point and is not encrypted beyond WiFi. A link key can
  authenticate the host.
