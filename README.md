# temoto
Setup package for TeMoto

## Installation Instructions

``` bash
# Go to your catkin workspace source dir
cd <catkin_ws>/src/

# Create a subfolder for maintaining TeMoto things
mkdir temoto && cd temoto

# Clone 'temoto'
git clone <link-to-this-repo>

# Run the TeMoto setup script & follow the instructions
# the tailing 'temoto' parameter indicates the installation dir
bash temoto/scripts/temoto_setup.sh temoto

# Install MeTA
cd temoto_nlp
bash scripts/install_meta.sh

# Compile your workspace
cd <catkin_ws> && catkin_make
```
.
