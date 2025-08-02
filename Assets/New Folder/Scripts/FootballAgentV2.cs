using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class FootballAgentV2 : Agent
{
    public GameManager gameManager;
    public Transform ball;
    public Transform opponent;
    public Transform goal_albert;
    public Transform goal_Kai;
    public Rigidbody ballRb;

    public float moveSpeed = 8f;
    public float rotateSpeed = 150f;
    public float kickPower = 3f;
    public float baseError = 0.1f;

    public float maxAccel = 10f; // acceleration power

    private Rigidbody rb;
    public Transform own_goal;
    public Transform opponent_goal;
    private bool hasControl = false;
    private float episode_timer;
    public float max_duration = 60f;

    public bool first_touch = false;
    private float prevDistToBall;


    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        
        if (this.name.Contains("Albert"))
        {
            own_goal = goal_albert;
            opponent_goal = goal_Kai;
        }
        else
        {
            own_goal = goal_Kai;
            opponent_goal = goal_albert;
        }
    }

    public override void OnEpisodeBegin()
    {
        prevDistToBall = Vector3.Distance(transform.position, ball.position);

        first_touch = false;
        episode_timer = 0f;
        Vector3 toBallDir = (ball.localPosition - own_goal.localPosition).normalized;
        transform.localPosition = own_goal.localPosition + toBallDir * 5f;
        transform.localPosition = new Vector3(transform.localPosition.x, 1.5f, transform.localPosition.z);
        transform.localRotation = Quaternion.LookRotation(toBallDir);
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Reset opponent (Kai) just in front of their goal
        if (opponent != null && opponent_goal != null)
        {
            Vector3 oppToBallDir = (ball.localPosition - opponent_goal.localPosition).normalized;
            opponent.localPosition = opponent_goal.localPosition + oppToBallDir * 5f;
            opponent.localPosition = new Vector3(opponent.localPosition.x, 1.5f, opponent.localPosition.z);
            opponent.localRotation = Quaternion.LookRotation(oppToBallDir);

            // Reset opponent velocity if needed (only if they have Rigidbody)
            Rigidbody opponentRb = opponent.GetComponent<Rigidbody>();
            if (opponentRb != null)
            {
                opponentRb.linearVelocity = Vector3.zero;
                opponentRb.angularVelocity = Vector3.zero;
            }
        }

        // Reset ball to center between goals
        Vector3 center = (own_goal.localPosition + opponent_goal.localPosition) / 2f;
        ball.localPosition = new Vector3(center.x, 0.5f, center.z);
        ballRb.linearVelocity = Vector3.zero;
        ballRb.angularVelocity = Vector3.zero;
    }


    public override void CollectObservations(VectorSensor sensor)
    {


        // Forward direction
        Vector3 forward = transform.forward.normalized;
        sensor.AddObservation(forward.x);
        sensor.AddObservation(forward.z);

        Vector3 relBallPos = transform.InverseTransformPoint(ball.position);
        sensor.AddObservation(relBallPos.x);
        sensor.AddObservation(relBallPos.z);

        Vector3 relBallOpp = ball.InverseTransformPoint(opponent.position);
        sensor.AddObservation(relBallOpp.x);
        sensor.AddObservation(relBallOpp.z);

        Vector3 relBallOwnGoal = ball.InverseTransformPoint(own_goal.position);
        sensor.AddObservation(relBallOwnGoal.x);
        sensor.AddObservation(relBallOwnGoal.z);

        Vector3 relBallOppGoal = ball.InverseTransformPoint(opponent_goal.position);
        sensor.AddObservation(relBallOppGoal.x);
        sensor.AddObservation(relBallOppGoal.z);

        sensor.AddObservation(ballRb.linearVelocity.x);
        
        sensor.AddObservation(ballRb.linearVelocity.y);

        Vector3 relOpp = transform.InverseTransformPoint(opponent.position);
        sensor.AddObservation(relOpp.x);
        sensor.AddObservation(relOpp.z);

        Vector3 relOwnGoal = transform.InverseTransformPoint(own_goal.position);
        sensor.AddObservation(relOwnGoal.x);
        sensor.AddObservation(relOwnGoal.z);

        Vector3 relOppGoal = transform.InverseTransformPoint(opponent_goal.position);
        sensor.AddObservation(relOppGoal.x);
        sensor.AddObservation(relOppGoal.z);
        


        

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        episode_timer += Time.deltaTime;
        float rotate = actions.ContinuousActions[0];
        float move = actions.ContinuousActions[1];
        float kick = actions.ContinuousActions[2];
        float control = actions.ContinuousActions[3];

        if (move > 0.5f)
        {
            rb.MovePosition(transform.position + transform.forward * moveSpeed * Time.fixedDeltaTime  *move);
            hasControl = false;
        }

        float rotationAmount = rotate * rotateSpeed * Time.fixedDeltaTime;
        transform.Rotate(Vector3.up, rotationAmount);


        float distToBall = Vector3.Distance(transform.position, ball.position);

        if (control > 0.5f && distToBall < 4.5f)
            hasControl = true;
        else if (control <= 0.5f || distToBall >= 1f)
            hasControl = false;

        if (hasControl)
        {
            Vector3 controlPoint = transform.position + transform.forward * 0.5f;
            ball.position = Vector3.Lerp(ball.position, controlPoint, 0.2f);
            ball.rotation = transform.rotation;
        }

        if (kick > 0.5f && distToBall < 4.5f)
        {
            float power = kick * kickPower;
            float error = power * baseError;
            Vector3 dir = (opponent_goal.position - transform.position).normalized;
            dir += Random.insideUnitSphere * error;
            ballRb.AddForce(dir.normalized * power, ForceMode.Impulse);
            hasControl = false;
        }
    

        if (episode_timer >= max_duration)
        {
            AddReward(-0.1f);
            Debug.Log("RESET");
            EndEpisode();

        }
        // float rotationPenalty = Mathf.Abs(rotate) * 0.001f;
        // AddReward(-rotationPenalty);
        float currentDistToBall = Vector3.Distance(transform.position, ball.position);
        float distanceDelta = prevDistToBall - currentDistToBall;

        if (distanceDelta > 0)
        {
            AddReward(0.002f * distanceDelta);  // Approached ball
        }
        // else{
        //     AddReward(0.002f * distanceDelta);
        // }
        Vector3 dirToball = (ball.position - transform.position).normalized;
        float alignment = Vector3.Dot(transform.forward, dirToball);
        AddReward(0.002f * alignment);


        prevDistToBall = currentDistToBall;


    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;

        // 0: Rotate
        float rotate = 0f;
        if (Input.GetKey(KeyCode.A)) rotate = -1f;
        if (Input.GetKey(KeyCode.D)) rotate = 1f;
        ca[0] = rotate;

        // 1: Move
        ca[1] = Input.GetKey(KeyCode.W) ? 1f : 0f;

        // 2: Kick
        ca[2] = Input.GetKey(KeyCode.Space) ? 1f : 0f;

        // 3: Control
        ca[3] = Input.GetKey(KeyCode.LeftShift) ? 1f : 0f;
       

    }
}

//force recompile