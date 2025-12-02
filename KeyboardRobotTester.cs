using UnityEngine;
using Preliy.Flange;
using UnityEngine.InputSystem;

public class KeyboardRobotTester : MonoBehaviour
{
    [SerializeField] private RobotPositionController robotController;
    [SerializeField] private float moveSpeed = 0.01f;
    [SerializeField] private float verticalMoveSpeed = 0.005f; // Slower for vertical movement
    
    // Position limits in millimeters
    private readonly float minX = -800f;
    private readonly float maxX = 800f;
    private readonly float minY = 0f;
    private readonly float maxY = 700f;
    private readonly float minZ = -1000f;
    private readonly float maxZ = -350f;
    
    private Keyboard keyboard;

    void Start()
    {
        if (robotController == null)
            robotController = GetComponent<RobotPositionController>();
            
        keyboard = Keyboard.current;
        
        if (robotController != null)
        {
            // Enable fixed Z rotation by default for XZ control
            robotController.SetFixZRotation(true);
            
            // Set starting position to (150, 400, -600) mm
            // Convert mm to meters (Unity uses meters)
            Vector3 startPosition = new Vector3(0.150f, 0.050f, -0.600f);
            robotController.MoveToPosition(startPosition);
            
            Debug.Log("🎮 Keyboard controller started");
            Debug.Log($"🎯 Starting position set to: X=150mm, Y=50mm, Z=-600mm");
            Debug.Log($"📏 Position limits - X: [{minX}, {maxX}]mm, Y: [{minY}, {maxY}]mm, Z: [{minZ}, {maxZ}]mm");
        }
    }

    void Update()
    {
        if (keyboard == null || robotController == null) return;
        
        HandleKeyboardInput();
    }

    private void HandleKeyboardInput()
    {
        // XZ Movement controls (horizontal plane)
        Vector3 movement = Vector3.zero;
        
        // Forward/Backward (Z-axis)
        if (keyboard.wKey.isPressed) movement.z += moveSpeed;
        if (keyboard.sKey.isPressed) movement.z -= moveSpeed;
        
        // Left/Right (X-axis)
        if (keyboard.aKey.isPressed) movement.x -= moveSpeed;
        if (keyboard.dKey.isPressed) movement.x += moveSpeed;
        
        // Up/Down (Y-axis) - optional, with slower speed
        if (keyboard.qKey.isPressed) movement.y += verticalMoveSpeed;
        if (keyboard.eKey.isPressed) movement.y -= verticalMoveSpeed;
        
        if (movement != Vector3.zero)
        {
            // Get current position and apply movement
            Vector3 currentPos = robotController.GetCurrentPosition();
            Vector3 newPos = currentPos + movement;
            
            // Apply limits (convert limits from mm to meters for comparison)
            newPos.x = Mathf.Clamp(newPos.x, minX / 1000f, maxX / 1000f);
            newPos.y = Mathf.Clamp(newPos.y, minY / 1000f, maxY / 1000f);
            newPos.z = Mathf.Clamp(newPos.z, minZ / 1000f, maxZ / 1000f);
            
            // Only move if position actually changed (not at boundary)
            if (newPos != currentPos)
            {
                robotController.MoveToPosition(newPos);
                
                // Check if we hit any boundaries
                CheckBoundaries(newPos);
            }
        }

        // Reset to starting position (150, 400, -600) mm
        if (keyboard.rKey.wasPressedThisFrame) 
        {
            Vector3 startPosition = new Vector3(0.150f, 0.400f, -0.600f);
            robotController.MoveToPosition(startPosition);
            Debug.Log("🔄 Reset to starting position: X=150mm, Y=400mm, Z=-600mm");
        }

        // Print current position and status
        if (keyboard.pKey.wasPressedThisFrame)
        {
            Vector3 pos = robotController.GetCurrentPosition();
            Vector3 rot = robotController.GetCurrentRotation();
            
            // Display in both meters and millimeters
            Vector3 posMm = pos * 1000f; // Convert to mm
            Debug.Log($"📊 Position: X={pos.x:F3}m ({posMm.x:F1}mm), Y={pos.y:F3}m ({posMm.y:F1}mm), Z={pos.z:F3}m ({posMm.z:F1}mm)");
            
            // Show distance to boundaries
            ShowBoundaryDistances(posMm);
        }

        // Increase movement speed (using equals key)
        if (keyboard.equalsKey.wasPressedThisFrame)
        {
            moveSpeed *= 1.5f;
            verticalMoveSpeed *= 1.5f;
            Debug.Log($"⚡ Speed increased to: {moveSpeed:F4} m/step");
        }

        // Decrease movement speed (using minus key)
        if (keyboard.minusKey.wasPressedThisFrame)
        {
            moveSpeed /= 1.5f;
            verticalMoveSpeed /= 1.5f;
            Debug.Log($"🐢 Speed decreased to: {moveSpeed:F4} m/step");
        }

        // Precision mode (hold Shift for finer control)
        if (keyboard.shiftKey.isPressed && (keyboard.wKey.isPressed || keyboard.sKey.isPressed || 
                                           keyboard.aKey.isPressed || keyboard.dKey.isPressed ||
                                           keyboard.qKey.isPressed || keyboard.eKey.isPressed))
        {
            Vector3 precisionMovement = Vector3.zero;
            float precisionSpeed = moveSpeed * 0.3f;
            float precisionVerticalSpeed = verticalMoveSpeed * 0.3f;
            
            if (keyboard.wKey.isPressed) precisionMovement.z += precisionSpeed;
            if (keyboard.sKey.isPressed) precisionMovement.z -= precisionSpeed;
            if (keyboard.aKey.isPressed) precisionMovement.x -= precisionSpeed;
            if (keyboard.dKey.isPressed) precisionMovement.x += precisionSpeed;
            if (keyboard.qKey.isPressed) precisionMovement.y += precisionVerticalSpeed;
            if (keyboard.eKey.isPressed) precisionMovement.y -= precisionVerticalSpeed;
            
            if (precisionMovement != Vector3.zero)
            {
                Vector3 currentPos = robotController.GetCurrentPosition();
                Vector3 newPos = currentPos + precisionMovement;
                
                // Apply limits
                newPos.x = Mathf.Clamp(newPos.x, minX / 1000f, maxX / 1000f);
                newPos.y = Mathf.Clamp(newPos.y, minY / 1000f, maxY / 1000f);
                newPos.z = Mathf.Clamp(newPos.z, minZ / 1000f, maxZ / 1000f);
                
                if (newPos != currentPos)
                {
                    robotController.MoveToPosition(newPos);
                    CheckBoundaries(newPos);
                }
            }
        }
    }

    private void CheckBoundaries(Vector3 position)
    {
        Vector3 posMm = position * 1000f;
        bool atBoundary = false;
        
        if (posMm.x <= minX + 0.1f || posMm.x >= maxX - 0.1f)
        {
            Debug.Log($"⚠️ X-axis boundary reached: {posMm.x:F1}mm");
            atBoundary = true;
        }
        if (posMm.y <= minY + 0.1f || posMm.y >= maxY - 0.1f)
        {
            Debug.Log($"⚠️ Y-axis boundary reached: {posMm.y:F1}mm");
            atBoundary = true;
        }
        if (posMm.z <= minZ + 0.1f || posMm.z >= maxZ - 0.1f)
        {
            Debug.Log($"⚠️ Z-axis boundary reached: {posMm.z:F1}mm");
            atBoundary = true;
        }
    }

    private void ShowBoundaryDistances(Vector3 positionMm)
    {
        float distXMin = positionMm.x - minX;
        float distXMax = maxX - positionMm.x;
        float distYMin = positionMm.y - minY;
        float distYMax = maxY - positionMm.y;
        float distZMin = positionMm.z - minZ;
        float distZMax = maxZ - positionMm.z;
        
        Debug.Log($"📏 Distance to boundaries - X: [-{distXMin:F1}, +{distXMax:F1}]mm, Y: [-{distYMin:F1}, +{distYMax:F1}]mm, Z: [-{distZMin:F1}, +{distZMax:F1}]mm");
    }

    // Public methods for other scripts to control the robot with limits
    public void MoveToXZ(float x, float z)
    {
        if (robotController != null)
        {
            Vector3 currentPos = robotController.GetCurrentPosition();
            Vector3 newPos = new Vector3(x, currentPos.y, z);
            
            // Apply limits
            newPos.x = Mathf.Clamp(newPos.x, minX / 1000f, maxX / 1000f);
            newPos.y = Mathf.Clamp(newPos.y, minY / 1000f, maxY / 1000f);
            newPos.z = Mathf.Clamp(newPos.z, minZ / 1000f, maxZ / 1000f);
            
            robotController.MoveToPosition(newPos);
        }
    }

    public void MoveToXZY(float x, float z, float y)
    {
        if (robotController != null)
        {
            Vector3 newPos = new Vector3(x, y, z);
            
            // Apply limits
            newPos.x = Mathf.Clamp(newPos.x, minX / 1000f, maxX / 1000f);
            newPos.y = Mathf.Clamp(newPos.y, minY / 1000f, maxY / 1000f);
            newPos.z = Mathf.Clamp(newPos.z, minZ / 1000f, maxZ / 1000f);
            
            robotController.MoveToPosition(newPos);
        }
    }

    public Vector2 GetXZPosition()
    {
        if (robotController != null)
        {
            Vector3 pos = robotController.GetCurrentPosition();
            return new Vector2(pos.x, pos.z);
        }
        return Vector2.zero;
    }

    public void SetMoveSpeed(float newSpeed)
    {
        moveSpeed = newSpeed;
        verticalMoveSpeed = newSpeed * 0.5f; // Keep vertical slower
    }
    
    // Public method to check if a position is within limits
    public bool IsPositionWithinLimits(Vector3 position)
    {
        Vector3 posMm = position * 1000f;
        return posMm.x >= minX && posMm.x <= maxX &&
               posMm.y >= minY && posMm.y <= maxY &&
               posMm.z >= minZ && posMm.z <= maxZ;
    }
}