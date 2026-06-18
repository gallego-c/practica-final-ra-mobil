"""Path setup so ctf_navigation Python modules can import each other."""
import os
import sys

import rospkg


def ensure_scripts_on_path():
    scripts = os.path.join(rospkg.RosPack().get_path('ctf_navigation'), 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)


ensure_scripts_on_path()
