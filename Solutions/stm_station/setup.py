from setuptools import setup    # Import the setup function from setuptools to configure the package
import os                       # Import the os module to handle paths
from glob import glob           # Import the glob module to include files using patterns

# Define the package name
package_name = 'stm_station'

# Call the setup function to configure the package
setup(
    name=package_name,                                      # Name of the package
    version='0.0.0',                                        # Version of the package
    packages=[package_name],                                # Python packages to include in the installation
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),                  # Add ament resource index file for the package
        ('share/' + package_name, ['package.xml']),         # Add the package.xml file to provide metadata about the package
    ],
    install_requires=['setuptools'],                        # Dependencies required for the package
    zip_safe=True,                                          # Ensures the package can be safely installed as a .zip file
    maintainer='athens2024ros',                             # Name of the maintainer
    maintainer_email='your_email@gmail.com',                # Email of the maintainer
    description='STM Station Motor control Package',        # Brief description of the package
    license='TODO: License declaration',                    # License type for the package (e.g., Apache 2.0, MIT)
    tests_require=['pytest'],                               # Specify the testing framework required for this package
    entry_points={                                          # Define the command-line executables and their corresponding Python entry points    
        'console_scripts': [
            'stm_serial_node = stm_station.stm_serial_node:main', 
        ],
    },  
)
