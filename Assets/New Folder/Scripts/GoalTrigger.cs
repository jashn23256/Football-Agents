using UnityEngine;

public class GoalTrigger : MonoBehaviour
{
    public bool isBlueGoal;
    public FootballAgentV2 blueAgent;
    public FootballAgentV2 redAgent;
    public GameManager gameManager; // optional centralized reset

    void OnTriggerEnter(Collider other)
    {
        // Debug.Log($"Trigger entered by: {other.name}");

        if (other.CompareTag("ball"))
        {
            Debug.Log("Goal scored!");

            if (isBlueGoal)
            {
                redAgent.AddReward(1.0f);
                blueAgent.AddReward(-1.0f);
            }
            else
            {
                blueAgent.AddReward(1.0f);
                redAgent.AddReward(-1.0f);
            }

            gameManager.ResetEnvironment();
        }
    }
}
