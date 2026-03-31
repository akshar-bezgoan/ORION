# ORION Development Aliases
# Source this file to get convenient shortcuts for ORION development

# Navigate to workspace
alias orion_ws='cd ~/Documents/ORION/orion_ws'

# Quick development workflow (fast rebuild + launch)
alias orion_dev='~/Documents/ORION/orion_ws/dev_workflow.sh dev'

# Full rebuild workflow
alias orion_full='~/Documents/ORION/orion_ws/dev_workflow.sh full'

# Just build
alias orion_build='~/Documents/ORION/orion_ws/dev_workflow.sh build'

# Just launch
alias orion_launch='~/Documents/ORION/orion_ws/dev_workflow.sh launch'

# Edit xacro file
alias orion_xacro='code ~/Documents/ORION/orion_ws/src/orion_description/xacro/orion.xacro'

# Edit launch file
alias orion_launch_edit='code ~/Documents/ORION/orion_ws/src/orion_description/launch/display.launch.py'

echo "ORION development aliases loaded!"
echo "Available commands:"
echo "  orion_dev     - Fast rebuild and launch (use after xacro changes)"
echo "  orion_full    - Complete rebuild and launch"
echo "  orion_build   - Just build the package"
echo "  orion_launch  - Just launch RViz2"
echo "  orion_ws      - Navigate to workspace"
echo "  orion_xacro   - Edit xacro file"
echo "  orion_launch_edit - Edit launch file"