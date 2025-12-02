using UnityEngine;
using Preliy.Flange;
using System.Reflection;
using System.Collections.Generic;

public class RobotPositionController : MonoBehaviour
{
    [Header("Robot Reference")]
    [SerializeField] private Controller robotController;
    
    [Header("Current Target")]
    [SerializeField] private Vector3 currentPosition = Vector3.zero;
    [SerializeField] private Vector3 currentRotation = Vector3.zero;
    
    [Header("Fixed Z Rotation")]
    [SerializeField] private bool fixZRotation = true;
    [SerializeField] private float fixedZAngle = 180f; // Always 180 degrees
    
    [Header("IK Settings")]
    [SerializeField] private int toolIndex = 0;
    [SerializeField] private int frameIndex = 0;
    // Removed unused showErrors variable
    
    [Header("Debug")]
    [SerializeField] private bool debugMode = true;
    
    // Store previous values to detect changes
    private Vector3 lastPosition;
    private Vector3 lastRotation;
    private bool valuesChanged = false;
    
    void Start()
    {
        if (robotController == null)
            robotController = GetComponent<Controller>();
        
        if (robotController == null)
        {
            Debug.LogError("No Controller found!");
            return;
        }
        
        // Initialize with fixed Z rotation
        if (fixZRotation)
        {
            currentRotation = new Vector3(currentRotation.x, currentRotation.y, fixedZAngle);
        }
        
        lastPosition = currentPosition;
        lastRotation = currentRotation;
        
        if (debugMode)
            Debug.Log("🤖 Robot Position Controller initialized with fixed Z rotation");
    }
    
    void Update()
    {
        // Apply fixed Z rotation if enabled
        if (fixZRotation && currentRotation.z != fixedZAngle)
        {
            currentRotation.z = fixedZAngle;
            valuesChanged = true;
        }
        
        // Check if values changed in the inspector
        if (currentPosition != lastPosition || currentRotation != lastRotation)
        {
            valuesChanged = true;
            lastPosition = currentPosition;
            lastRotation = currentRotation;
        }
        
        // If values changed, execute movement
        if (valuesChanged)
        {
            ExecuteMovement();
            valuesChanged = false;
        }
    }
    
    public void MoveToPosition(float x, float y, float z)
    {
        MoveToPosition(new Vector3(x, y, z));
    }
    
    public void MoveToPosition(Vector3 position)
    {
        currentPosition = position;
        ExecuteMovement();
    }
    
    public void MoveToPose(Vector3 position, Vector3 rotationEuler)
    {
        currentPosition = position;
        SetRotation(rotationEuler);
    }
    
    public void MoveToPose(Vector3 position, Quaternion rotation)
    {
        currentPosition = position;
        SetRotation(rotation.eulerAngles);
    }
    
    private void ExecuteMovement()
    {
        if (robotController == null || !robotController.IsValid.Value)
        {
            Debug.LogWarning("Robot controller not ready!");
            return;
        }
        
        // Apply fixed Z rotation before creating the pose
        Vector3 finalRotation = currentRotation;
        if (fixZRotation)
        {
            finalRotation.z = fixedZAngle;
        }
        
        // Create the target pose with fixed Z rotation
        Quaternion rotation = Quaternion.Euler(finalRotation);
        Matrix4x4 targetPose = Matrix4x4.TRS(currentPosition, rotation, Vector3.one);
        
        // Create CartesianTarget
        var cartesianTarget = new CartesianTarget(
            targetPose, 
            Configuration.Default, 
            ExtJoint.Default
        );
        
        if (debugMode)
            Debug.Log($"🎯 Computing IK for position: {currentPosition} | Rotation: {finalRotation} (Z fixed to {fixedZAngle}°)");
        
        // Use the solver to compute inverse kinematics
        bool success = ComputeAndMoveToTarget(cartesianTarget);
            
        if (!success && debugMode)
            Debug.LogError("❌ IK computation failed - cannot reach target position!");
    }
    
    private bool ComputeAndMoveToTarget(CartesianTarget target)
    {
        try
        {
            var solver = robotController.Solver;
            if (solver == null)
            {
                Debug.LogError("Solver is null!");
                return false;
            }
            
            // Get the ignore mask - you might need to adjust this based on your needs
            var ignoreMask = GetIgnoreMask();
            
            // Method 1: Try ComputeInverse first (gets single solution)
            var ikSolution = solver.ComputeInverse(target, toolIndex, frameIndex, ignoreMask);
            
            if (ikSolution != null && ikSolution.IsValid)
            {
                if (debugMode) 
                    Debug.Log($"✅ IK Solution found: {ikSolution.JointTarget}");
                
                // Move the robot using the joint solution
                return MoveToJointTarget(ikSolution.JointTarget);
            }
            
            // Method 2: If single solution fails, try getting all solutions
            if (debugMode) 
                Debug.Log("🔄 Single solution failed, trying all solutions...");
            
            var allSolutions = solver.GetAllSolutions(target, toolIndex, frameIndex, true, ignoreMask);
            
            if (allSolutions != null && allSolutions.Count > 0)
            {
                // Find the first valid solution
                foreach (var solution in allSolutions)
                {
                    if (solution.IsValid)
                    {
                        if (debugMode) 
                            Debug.Log($"✅ Found valid solution from {allSolutions.Count} total solutions: {solution.JointTarget}");
                        
                        return MoveToJointTarget(solution.JointTarget);
                    }
                }
            }
            
            Debug.LogError("❌ No valid IK solutions found for target position!");
            return false;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ IK computation failed: {e.Message}");
            return false;
        }
    }
    
    private bool MoveToJointTarget(JointTarget jointTarget)
    {
        try
        {
            var mechGroup = robotController.MechanicalGroup;
            if (mechGroup == null)
            {
                Debug.LogError("MechanicalGroup is null!");
                return false;
            }
            
            // Use the SetJoints method to move the robot
            mechGroup.SetJoints(jointTarget, true); // true = notify about changes
            
            if (debugMode)
                Debug.Log($"🚀 Robot moving to joint target: {jointTarget}");
            
            return true;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"❌ Failed to set joint target: {e.Message}");
            return false;
        }
    }
    
    private SolutionIgnoreMask GetIgnoreMask()
    {
        // Create a default ignore mask - adjust based on your needs
        // This controls which aspects of the solution to ignore
        var ignoreMask = new SolutionIgnoreMask();
        
        // You might need to set these flags based on your requirements:
        // ignoreMask.IgnoreConfiguration = true;
        // ignoreMask.IgnoreLinearAxes = true; 
        // ignoreMask.IgnoreRotationAxes = true;
        
        return ignoreMask;
    }
    
    // Utility methods
    public void MoveRelative(float x, float y, float z)
    {
        currentPosition += new Vector3(x, y, z);
        ExecuteMovement();
    }
    
    public void SetRotation(float x, float y, float z)
    {
        // Apply fixed Z rotation if enabled
        if (fixZRotation)
        {
            currentRotation = new Vector3(x, y, fixedZAngle);
        }
        else
        {
            currentRotation = new Vector3(x, y, z);
        }
        ExecuteMovement();
    }
    
    public void SetRotation(Vector3 rotation)
    {
        SetRotation(rotation.x, rotation.y, rotation.z);
    }
    
    public void SetToolIndex(int index)
    {
        toolIndex = Mathf.Clamp(index, 0, robotController.Tools.Count - 1);
    }
    
    public void SetFrameIndex(int index)
    {
        frameIndex = Mathf.Clamp(index, 0, robotController.Frames.Count - 1);
    }
    
    public void SetFixZRotation(bool enable)
    {
        fixZRotation = enable;
        if (enable)
        {
            currentRotation.z = fixedZAngle;
            ExecuteMovement();
        }
    }
    
    public void SetFixedZAngle(float zAngle)
    {
        fixedZAngle = zAngle;
        if (fixZRotation)
        {
            currentRotation.z = fixedZAngle;
            ExecuteMovement();
        }
    }
    
    public Vector3 GetCurrentPosition() => currentPosition;
    public Vector3 GetCurrentRotation() => currentRotation;
    
    // This will be called when values change in the inspector
    private void OnValidate()
    {
        if (Application.isPlaying && enabled)
        {
            // Apply fixed Z rotation when values change in inspector
            if (fixZRotation && currentRotation.z != fixedZAngle)
            {
                currentRotation.z = fixedZAngle;
            }
            valuesChanged = true;
        }
    }
    
    [ContextMenu("Test Movement Now")]
    public void TestMovement()
    {
        Debug.Log("🧪 Manual movement test triggered!");
        ExecuteMovement();
    }
    
    [ContextMenu("Print Current Status")]
    public void PrintStatus()
    {
        Debug.Log($"📊 Current Status - Position: {currentPosition}, Rotation: {currentRotation}");
        Debug.Log($"🔧 Tool: {toolIndex}, Frame: {frameIndex}");
        Debug.Log($"🎯 Fixed Z Rotation: {fixZRotation} ({fixedZAngle}°)");
        Debug.Log($"🤖 Controller Valid: {robotController?.IsValid.Value}");
    }
    
    [ContextMenu("Test Reachable Position")]
    public void TestReachablePosition()
    {
        // Test with a position that should be reachable
        currentPosition = new Vector3(0.4f, 0.2f, 0.3f);
        ExecuteMovement();
    }
    
    [ContextMenu("Toggle Fixed Z Rotation")]
    public void ToggleFixedZRotation()
    {
        SetFixZRotation(!fixZRotation);
        Debug.Log($"🔄 Fixed Z Rotation: {(fixZRotation ? "ENABLED" : "DISABLED")}");
    }
}