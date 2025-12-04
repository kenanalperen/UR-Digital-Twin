using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimpleRosController : MonoBehaviour
{
    [Header("Robot Reference")]
    [SerializeField] private RobotPositionController robotController;
    
    [Header("ROS Topics")]
    [SerializeField] private string positionCommandTopic = "/unity_robot/position_command";
    
    [Header("Position Limits (meters)")]
    [SerializeField] private float minX = -0.800f;  // -800 mm
    [SerializeField] private float maxX = 0.800f;   // 800 mm
    [SerializeField] private float minY = 0.000f;   // 0 mm
    [SerializeField] private float maxY = 0.700f;   // 700 mm
    [SerializeField] private float minZ = -1.000f;  // -1000 mm
    [SerializeField] private float maxZ = -0.350f;  // -350 mm
    
    private ROSConnection ros;
    
    void Start()
    {
        // Get the RobotPositionController if not assigned
        if (robotController == null)
            robotController = GetComponent<RobotPositionController>();
        
        if (robotController == null)
        {
            Debug.LogError("❌ SimpleRosController: No RobotPositionController found!");
            return;
        }
        
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to position command topic
        ros.Subscribe<Vector3Msg>(positionCommandTopic, PositionCommandCallback);
        
        Debug.Log($"🤖 Simple ROS Controller initialized");
        Debug.Log($"   - Listening to: {positionCommandTopic}");
        Debug.Log($"   - Just send Vector3 messages to move the robot!");
        Debug.Log($"   - Example: ros2 topic pub {positionCommandTopic} geometry_msgs/msg/Vector3 \"{{x: 0.150, y: 0.400, z: -0.600}}\"");
        
        // Enable fixed Z rotation (same as before)
        robotController.SetFixZRotation(true);
    }
    
    private void PositionCommandCallback(Vector3Msg msg)
    {
        // Convert ROS Vector3 to Unity Vector3 (meters)
        Vector3 targetPosition = new Vector3(
            (float)msg.x,
            (float)msg.y,
            (float)msg.z
        );
        
        // Apply limits
        Vector3 clampedPosition = new Vector3(
            Mathf.Clamp(targetPosition.x, minX, maxX),
            Mathf.Clamp(targetPosition.y, minY, maxY),
            Mathf.Clamp(targetPosition.z, minZ, maxZ)
        );
        
        // Log the command
        Vector3 targetMm = targetPosition * 1000f;
        Vector3 clampedMm = clampedPosition * 1000f;
        
        Debug.Log($"🎯 Received position command:");
        Debug.Log($"   Requested: X={targetMm.x:F1}mm, Y={targetMm.y:F1}mm, Z={targetMm.z:F1}mm");
        
        if (clampedPosition != targetPosition)
        {
            Debug.Log($"⚠️  Clamped to: X={clampedMm.x:F1}mm, Y={clampedMm.y:F1}mm, Z={clampedMm.z:F1}mm");
        }
        
        // Move the robot
        robotController.MoveToPosition(clampedPosition);
    }
    
    [ContextMenu("Test Move to (150, 400, -600) mm")]
    public void TestMoveToStart()
    {
        Vector3 position = new Vector3(0.150f, 0.400f, -0.600f);
        robotController.MoveToPosition(position);
        Debug.Log($"🧪 Test: Moving to X=150mm, Y=400mm, Z=-600mm");
    }
    
    [ContextMenu("Print Current Position")]
    public void PrintCurrentPosition()
    {
        if (robotController == null) return;
        
        Vector3 position = robotController.GetCurrentPosition();
        Vector3 posMm = position * 1000f;
        
        Debug.Log($"📊 Current Position:");
        Debug.Log($"   Meters: ({position.x:F3}, {position.y:F3}, {position.z:F3})");
        Debug.Log($"   Millimeters: ({posMm.x:F1}, {posMm.y:F1}, {posMm.z:F1})");
    }
}
