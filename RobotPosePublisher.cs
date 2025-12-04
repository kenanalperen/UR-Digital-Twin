using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotPositionPublisher : MonoBehaviour
{
    [Header("Robot Reference")]
    [SerializeField] private RobotPositionController robotController;
    
    [Header("ROS Settings")]
    [SerializeField] private string positionTopic = "/unity_robot/current_position";
    [SerializeField] private float publishRate = 10f; // Hz
    
    [Header("Debug")]
    [SerializeField] private bool debugMode = true;
    [SerializeField] private float debugPrintInterval = 1f; // Print every 1 second
    
    private ROSConnection ros;
    private float timeSinceLastPublish = 0f;
    private float timeSinceLastDebug = 0f;
    
    void Start()
    {
        // Get the RobotPositionController if not assigned
        if (robotController == null)
            robotController = GetComponent<RobotPositionController>();
        
        if (robotController == null)
        {
            Debug.LogError("❌ RobotPositionPublisher: No RobotPositionController found!");
            return;
        }
        
        // Ensure publish rate is valid
        if (publishRate <= 0f)
        {
            Debug.LogWarning("⚠️ Publish rate must be positive. Setting to 10 Hz.");
            publishRate = 10f;
        }
        
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register position publisher
        ros.RegisterPublisher<Vector3Msg>(positionTopic);
        
        Debug.Log($"🤖 RobotPositionPublisher initialized");
        Debug.Log($"   - Topic: {positionTopic}");
        Debug.Log($"   - Rate: {publishRate} Hz");
        
        // Print initial position
        Vector3 initialPos = robotController.GetCurrentPosition();
        Vector3 posMm = initialPos * 1000f;
        Debug.Log($"   - Initial Position: ({posMm.x:F1}, {posMm.y:F1}, {posMm.z:F1}) mm");
    }
    
    void Update()
    {
        if (robotController == null) return;
        
        // Update timers
        timeSinceLastPublish += Time.deltaTime;
        timeSinceLastDebug += Time.deltaTime;
        
        // Publish at the specified rate
        if (timeSinceLastPublish >= 1f / publishRate)
        {
            PublishCurrentPosition();
            timeSinceLastPublish = 0f;
        }
        
        // Debug output at fixed interval
        if (debugMode && timeSinceLastDebug >= debugPrintInterval)
        {
            PrintDebugInfo();
            timeSinceLastDebug = 0f;
        }
    }
    
    private void PublishCurrentPosition()
    {
        Vector3 position = robotController.GetCurrentPosition();
        
        try
        {
            // Create and publish position message
            Vector3Msg positionMsg = new Vector3Msg
            {
                x = position.x,
                y = position.y,
                z = position.z
            };
            
            ros.Publish(positionTopic, positionMsg);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to publish position: {e.Message}");
        }
    }
    
    private void PrintDebugInfo()
    {
        Vector3 position = robotController.GetCurrentPosition();
        Vector3 posMm = position * 1000f;
        
        Debug.Log($"📤 Position - X: {posMm.x:F1}mm, Y: {posMm.y:F1}mm, Z: {posMm.z:F1}mm");
    }
    
    [ContextMenu("Publish Current Position")]
    public void PublishNow()
    {
        if (robotController == null) return;
        
        Vector3 position = robotController.GetCurrentPosition();
        Vector3 posMm = position * 1000f;
        
        Debug.Log($"📤 Manual publish:");
        Debug.Log($"   Position: ({position.x:F3}, {position.y:F3}, {position.z:F3}) m");
        Debug.Log($"   Position: ({posMm.x:F1}, {posMm.y:F1}, {posMm.z:F1}) mm");
        
        PublishCurrentPosition();
    }
    
    [ContextMenu("Print Current Position")]
    public void PrintCurrentPosition()
    {
        if (robotController == null) return;
        
        Vector3 position = robotController.GetCurrentPosition();
        Vector3 posMm = position * 1000f;
        
        Debug.Log($"🤖 Current Robot Position:");
        Debug.Log($"   Meters: ({position.x:F3}, {position.y:F3}, {position.z:F3})");
        Debug.Log($"   Millimeters: ({posMm.x:F1}, {posMm.y:F1}, {posMm.z:F1})");
        Debug.Log($"   Topic: {positionTopic}");
        Debug.Log($"   Publish Rate: {publishRate} Hz");
    }
    
    [ContextMenu("Test ROS Connection")]
    public void TestRosConnection()
    {
        if (ros == null)
        {
            Debug.LogError("❌ ROS Connection not initialized!");
            return;
        }
        
        Debug.Log($"🔗 ROS Connection Status:");
        Debug.Log($"   - ROS IP: {ros.RosIPAddress}");
        Debug.Log($"   - ROS Port: {ros.RosPort}");
        Debug.Log($"   - Topic: {positionTopic}");
    }
}
