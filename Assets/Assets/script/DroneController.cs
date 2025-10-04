using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    [Header("Speed / Accel Limits")]
    public float maxHorizontalSpeed = 15f;
    public float maxClimbRate = 5f;         // m/s up/down
    public float horizontalAccel = 30f;     // m/s^2
    public float yawRateDegPerSec = 120f;

    [Header("Flight Area (optional)")]
    [Tooltip("경계/고도 제한을 적용할지 여부")]
    public bool limitArea = false;

    [Tooltip("허용 고도 범위 (limitArea=true일 때만 적용)")]
    public float minAltitude = 0f;
    public float maxAltitude = 500f;

    [Tooltip("허용 X 범위 (limitArea=true일 때만 적용)")]
    public Vector2 xBounds = new Vector2(-500f, 500f);

    [Tooltip("허용 Z 범위 (limitArea=true일 때만 적용)")]
    public Vector2 zBounds = new Vector2(-500f, 500f);

    Rigidbody rb;
    Vector3 desiredVelLocal;   // (strafe, up, forward) in local frame
    float desiredYawRate;      // deg/s

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.drag = 1.0f;
    }

    /// <summary>
    /// inputLocal.x = strafe (right +), inputLocal.y = climb (up +), inputLocal.z = forward (+)
    /// yawRateCmdDeg = desired yaw rate in deg/s
    /// </summary>
    public void SetCommand(Vector3 inputLocal, float yawRateCmdDeg)
    {
        // Clamp inputs to limits
        float str = Mathf.Clamp(inputLocal.x, -maxHorizontalSpeed, maxHorizontalSpeed);
        float up = Mathf.Clamp(inputLocal.y, -maxClimbRate, maxClimbRate);
        float fwd = Mathf.Clamp(inputLocal.z, -maxHorizontalSpeed, maxHorizontalSpeed);

        desiredVelLocal = new Vector3(str, up, fwd);
        desiredYawRate = Mathf.Clamp(yawRateCmdDeg, -yawRateDegPerSec, yawRateDegPerSec);
    }

    void FixedUpdate()
    {
        // --- Yaw (Rigidbody-friendly) ---
        Quaternion dq = Quaternion.AngleAxis(desiredYawRate * Time.fixedDeltaTime, Vector3.up);
        rb.MoveRotation(dq * rb.rotation);

        // --- Horizontal velocity control (x,z) ---
        Vector3 desiredWorldVel = transform.TransformDirection(new Vector3(desiredVelLocal.x, 0f, desiredVelLocal.z));
        Vector3 currentWorldVel = rb.velocity;
        Vector3 horizVel = new Vector3(currentWorldVel.x, 0f, currentWorldVel.z);

        Vector3 horizErr = desiredWorldVel - horizVel;
        Vector3 horizAcc = Vector3.ClampMagnitude(horizErr * 1f, horizontalAccel); // P-제어
        rb.AddForce(horizAcc, ForceMode.Acceleration);

        // --- Vertical (y) ---
        float climbErr = desiredVelLocal.y - currentWorldVel.y;
        float climbAcc = Mathf.Clamp(climbErr * 1f, -horizontalAccel, horizontalAccel);
        rb.AddForce(Vector3.up * climbAcc, ForceMode.Acceleration);

        // --- Optional bounds ---
        if (limitArea)
        {
            Vector3 p = rb.position;

            // altitude
            p.y = Mathf.Clamp(p.y, minAltitude, maxAltitude);

            // XY bounds
            p.x = Mathf.Clamp(p.x, xBounds.x, xBounds.y);
            p.z = Mathf.Clamp(p.z, zBounds.x, zBounds.y);

            rb.MovePosition(p);
        }
    }

    public Vector3 CurrentVelocity() => rb.velocity;
    public float Altitude() => rb.position.y;

#if UNITY_EDITOR
    // 인스펙터에서 경계 시각화
    void OnDrawGizmosSelected()
    {
        if (!limitArea) return;

        // 바닥 박스
        Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
        float width = Mathf.Abs(xBounds.y - xBounds.x);
        float depth = Mathf.Abs(zBounds.y - zBounds.x);
        Vector3 center = new Vector3((xBounds.x + xBounds.y) * 0.5f, (minAltitude + maxAltitude) * 0.5f, (zBounds.x + zBounds.y) * 0.5f);
        Vector3 size = new Vector3(width, Mathf.Abs(maxAltitude - minAltitude), depth);
        Gizmos.DrawWireCube(center, size);
    }
#endif
}
    