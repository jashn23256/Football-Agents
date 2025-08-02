using UnityEngine;

public class BallController : MonoBehaviour
{
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous; // helps with fast collision detection
    }

    void FixedUpdate()
    {
        rb.linearVelocity = Vector3.ClampMagnitude(rb.linearVelocity, 10f); // correctly clamp speed
    }
}
