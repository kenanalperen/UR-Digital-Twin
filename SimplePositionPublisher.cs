using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimplePositionPublisher : MonoBehaviour
{
    [Header("Robot Reference")]
    [SerializeField] private RobotPositionController robotController;
    
    [Header("ROS Topics")]
    [SerializeField] private string positionTopic = "/unity_robot/current_position";
    [SerializeField] private float publishRate = 5f; // Hz
    
    private ROSConnection ros;
    private float timeSinceLastPublish = 0f;
    
    void Start()
    {
        if (robotController == null)
            robotController = GetComponent<RobotPositionController>();
        
        if (robotController == null)
        {
            Debug.LogError("❌ SimplePositionPublisher: No RobotPositionController found!");
            return;
        }
        
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register position publisher
        ros.RegisterPublisher<Vector3Msg>(positionTopic);
        
        Debug.Log($"📤 Simple Position Publisher initialized");
        Debug.Log($"   - Publishing to: {positionTopic}");
        Debug.Log($"   - Rate: {publishRate} Hz");
    }
    
    void Update()
    {
        if (robotController == null) return;
        
        // Publish at the specified rate
        timeSinceLastPublish += Time.deltaTime;
        if (timeSinceLastPublish >= 1f / publishRate)
        {
            PublishPosition();
            timeSinceLastPublish = 0f;
        }
    }
    
    private void PublishPosition()
    {
        Vector3 position = robotController.GetCurrentPosition();
        
        try
        {
            Vector3Msg positionMsg = new Vector3Msg
            {
                x = position.x,
                y = position.y,
                z = position.z
            };
            
            ros.Publish(positionTopic, positionMsg);
            
            // Optional: Print occasionally (every 2 seconds)
            if (Time.frameCount % (int)(2f * publishRate) == 0)
            {
                Vector3 posMm = position * 1000f;
                Debug.Log($"📤 Published: X={posMm.x:F1}mm, Y={posMm.y:F1}mm, Z={posMm.z:F1}mm");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to publish: {e.Message}");
        }
    }
}
