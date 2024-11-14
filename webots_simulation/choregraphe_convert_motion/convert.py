import xml.etree.ElementTree as ET
import math

def convert_xar_to_motion(xar_file_path, motion_file_path):
    # Define namespace prefix
    ns = '{http://www.ald.softbankrobotics.com/schema/choregraphe/project.xsd}'
    
    # Predefined joint order
    desired_joint_order = [
        "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", 
        "LElbowYaw", "LElbowRoll", "LHipYawPitch", "LHipRoll", 
        "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", 
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", 
        "RAnklePitch", "RAnkleRoll", "RShoulderPitch", "RShoulderRoll", 
        "RElbowYaw", "RElbowRoll"
    ]
    
    # Parse the XML file
    tree = ET.parse(xar_file_path)
    root = tree.getroot()
    
    # Initialize joint and frame data
    joint_data = {name: {} for name in desired_joint_order}
    frames_set = set()  # Used to record all existing frame numbers
    
    # Find all ActuatorCurve nodes
    for actuator_curve in root.findall(f".//{ns}ActuatorList/{ns}ActuatorCurve"):
        joint_name = actuator_curve.get('actuator')
        
        # Process if the joint is in the predefined list
        if joint_name in joint_data:
            # Iterate over Key nodes in ActuatorCurve
            for key in actuator_curve.findall(f"{ns}Key"):
                frame = int(key.get('frame'))
                value = float(key.get('value'))
                
                # Convert angle to radians
                radians_value = math.radians(value)
                
                # Store the joint's radian value at this frame
                joint_data[joint_name][frame] = radians_value
                frames_set.add(frame)  # Record this frame number
    
    # Sort frame numbers to ensure ordered writing
    frames_list = sorted(frames_set)
    
    # Create Webots Motion format file
    with open(motion_file_path, 'w') as motion_file:
        # Write header
        motion_file.write(f"#WEBOTS_MOTION,V1.0,{','.join(desired_joint_order)}\n")
        
        # Write data for each frame in frames_list
        pose_counter = 1  # For continuous pose numbering
        for frame in frames_list:
            # Calculate timestamp, assuming 40 ms per frame (25 frames/second)
            total_milliseconds = frame * 40
            milliseconds = total_milliseconds % 1000
            seconds = (total_milliseconds // 1000) % 60
            minutes = (total_milliseconds // 1000) // 60
            
            # Format timestamp as MM:SS:SSS
            timestamp = f"{minutes:02}:{seconds:02}:{milliseconds:03}"
            pose_name = f"Pose{pose_counter}"
            
            # Construct joint data for this frame
            frame_data = []
            for joint_name in desired_joint_order:
                # Use value if joint has data for this frame; otherwise, fill 0.0
                radians_value = joint_data[joint_name].get(frame, 0.0)
                frame_data.append(f"{radians_value:.3f}")
            
            # Write one line of data
            motion_file.write(f"{timestamp},{pose_name}," + ",".join(frame_data) + "\n")
            
            # Increment pose counter
            pose_counter += 1

# Change pathes
xar_file = 'kickToRight\\behavior_1\\behavior.xar'
motion_file = 'kickToRight.motion'
convert_xar_to_motion(xar_file, motion_file)
