#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

from antrobot_ros.rdrive import RDrive


def main():
    """Simple script to disable RDrive."""
    print("RDrive Disable Script")
    print("====================")
    
    try:
        # Instantiate RDrive
        drive = RDrive()
        
        # Stop any motion first
        print("Stopping RDrive motion...")
        drive.cmd_vel(0.0, 0.0)
        print("✓ RDrive motion stopped")
        
        # Disable the drive system
        print("Disabling RDrive...")
        success = drive.disable()
        
        if success:
            print("✓ RDrive disabled successfully")
        else:
            print("✗ Failed to disable RDrive")
            
    except Exception as e:
        print(f"✗ Error disabling RDrive: {str(e)}")
    
    print("RDrive disable operation completed.")


if __name__ == '__main__':
    main()
