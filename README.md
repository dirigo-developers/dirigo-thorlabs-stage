# dirigo-thorlabs-stage
`dirigo-thorlabs-stage` provides Thorlabs motorized stage functionality in [Dirigo](https://dirigo.readthedocs.io/). 

> **Note**  
> This is a hardware plugin for Dirigo and is not intended to be used as a standalone library. 

[![PyPI](https://img.shields.io/pypi/v/dirigo-thorlabs-stage)](https://pypi.org/project/dirigo-thorlabs-stage/)


## Supported models
- MLS203 via BBD102/BBD202 (BBD302 coming soon)
- MCM3000


## Installation
First install [Thorlabs Kinesis® software](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=10285). Be sure to install Kinesis to its default location, `C:\Program Files\Thorlabs\Kinesis`. Then run:

```bash
pip install dirigo-thorlabs-stage
```

Verify that your device is recognized in the Kinesis app and functioning before using this plugin.


## Legal Disclaimer
This library is provided "as is" without any warranties, express or implied, including but not limited to the implied warranties of merchantability, fitness for a particular purpose, or non-infringement. The authors are not responsible for any damage to hardware, data loss, or other issues arising from the use or misuse of this library. Users are advised to thoroughly test this library with their specific hardware and configurations before deployment.

This library depends on the Thorlabs Kinesis® API and associated drivers, which must be installed and configured separately. Compatibility and performance depend on the proper installation and operation of these third-party components.

This library is an independent implementation based on publicly available documentation from Thorlabs. It is not affiliated with, endorsed by, or officially supported by Thorlabs.

Use this library at your own risk. Proper operation of hardware and compliance with applicable laws and regulations is the sole responsibility of the user.

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.


## Funding
Development has been supported in part by the National Cancer Institute of the National Institutes of Health under award number R01CA249151.

The content of this repository is solely the responsibility of the authors and does not necessarily represent the official views of the National Institutes of Health.
