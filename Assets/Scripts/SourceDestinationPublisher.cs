using System;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;
    public static readonly string[] LinkNames =
    {
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link",
        "/robot_upper_arm_link",
        "/robot_forearm_link",
        "/robot_wrist_1_link",
        "/robot_wrist_2_link",
        "/robot_wrist_3_link"
    };

    const int k_NumRobotGripperJoints = 4;
    public static readonly string[] GripperPartNames =
    {
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link/robot_upper_arm_link/robot_forearm_link/robot_wrist_1_link/robot_wrist_2_link/robot_wrist_3_link/robot_flange/robot_tool0/gripper_onrobot_rg2_base_link/gripper_right_inner_knuckle",
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link/robot_upper_arm_link/robot_forearm_link/robot_wrist_1_link/robot_wrist_2_link/robot_wrist_3_link/robot_flange/robot_tool0/gripper_onrobot_rg2_base_link/gripper_right_outer_knuckle",
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link/robot_upper_arm_link/robot_forearm_link/robot_wrist_1_link/robot_wrist_2_link/robot_wrist_3_link/robot_flange/robot_tool0/gripper_onrobot_rg2_base_link/gripper_left_inner_knuckle",
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link/robot_upper_arm_link/robot_forearm_link/robot_wrist_1_link/robot_wrist_2_link/robot_wrist_3_link/robot_flange/robot_tool0/gripper_onrobot_rg2_base_link/gripper_left_outer_knuckle"
    };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur10e_rg2_joints";

    [SerializeField]
    GameObject m_UR10e;

    [SerializeField]
    GameObject m_Target;

    [SerializeField]
    GameObject m_TargetPlacement;

    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // Robot Gripper Joints
    UrdfJointRevolute[] m_GripperJointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<Ur10eMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            var jointTransform = m_UR10e.transform.Find(linkName);
            if (jointTransform != null)
            {
                m_JointArticulationBodies[i] = jointTransform.GetComponent<UrdfJointRevolute>();
                if (m_JointArticulationBodies[i] == null)
                {
                    Debug.LogError($"UrdfJointRevolute component missing on {linkName}");
                }
            }
            else
            {
                Debug.LogError($"Transform not found: {linkName}");
            }
        }

        m_GripperJointArticulationBodies = new UrdfJointRevolute[k_NumRobotGripperJoints];

        for (var i = 0; i < k_NumRobotGripperJoints; i++)
        {
            var gripperJointTransform = m_UR10e.transform.Find(GripperPartNames[i]);
            if (gripperJointTransform != null)
            {
                m_GripperJointArticulationBodies[i] = gripperJointTransform.GetComponent<UrdfJointRevolute>();
                if (m_GripperJointArticulationBodies[i] == null)
                {
                    Debug.LogError($"UrdfJointRevolute component missing on {GripperPartNames[i]}");
                }
            }
            else
            {
                Debug.LogError($"Transform not found: {GripperPartNames[i]}");
            }
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            if (m_JointArticulationBodies[i] != null)
            {
                sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
            }
            else
            {
                Debug.LogError("Joint at index " + i + " is null. Please check joint setup.");
            }
        }

        sourceDestinationMessage.joints = new double[k_NumRobotJoints];
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = m_TargetPlacement.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}