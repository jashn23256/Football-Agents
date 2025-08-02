using UnityEngine;

public class GameManager : MonoBehaviour
{
    public FootballAgentV2 blueAgent;
    public FootballAgentV2 redAgent;
    public Transform ball;
    public Rigidbody ballRb;

    public void ResetEnvironment()
    {
        blueAgent.OnEpisodeBegin();
        redAgent.OnEpisodeBegin();
    }
}
