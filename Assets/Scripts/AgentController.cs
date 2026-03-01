using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class AgentController : Agent
{
    Rigidbody rb;
    [SerializeField] public bool inHumanControl;

    GameObject goal, trainingArea, environment, player;
    GameObject[] obstacles;
    public int obsCount = 1;
    public GameObject obstacle;

    Vector3 startingPosition;
    float lastDist;
    float move = 0;
    float turn = 0;

    Stats stats;

    const float MAX_MAGNITUDE = 0.5f;

    private float moveForce = 0.25f;
    private float turnForce = 100.0f; // Increased for better responsiveness

    // Constants for rewards
    private float deathReward = -5.0f;
    private float failReward = -3.0f;
    private float goalReward = 5.0f;

    public int episodeNum = 0;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        stats = GameObject.FindGameObjectWithTag("Stats").GetComponent<Stats>();

        trainingArea = transform.parent.gameObject;
        if (trainingArea.name != "TrainingAreaGrid") trainingArea = trainingArea.transform.parent.gameObject;

        environment = trainingArea.transform.Find("Environment").gameObject;
        player = trainingArea.transform.Find("Player").gameObject;
        goal = trainingArea.transform.Find("Goal").gameObject;

        startingPosition = transform.localPosition;

        obstacles = new GameObject[obsCount];
        for (int i = 0; i < obsCount; i++)
        {
            var pos = new Vector3(0, 10000, 0);
            obstacles[i] = Instantiate(obstacle, pos, Quaternion.identity, environment.transform);
        }
    }

    public Vector3 FindSafePos(Vector3 areaCenter, float y, GameObject avoidObject = null, float minDistance = 3.0f)
    {
        int attempts = 0;
        while (attempts < 20) // Increased attempts for tighter layouts
        {
            // Generate random position within the 18x18 floor (adjust ranges as needed)
            Vector3 pos = new Vector3(Random.Range(-9.0f, 9.0f), y, Random.Range(-9.0f, 9.0f));
        
            // 1. Check for physical overlap with walls/obstacles
            var colliders = Physics.OverlapBox(pos + areaCenter, new Vector3(0.5f, 0.25f, 0.5f));
        
            // 2. Check for distance from the 'avoidObject' (e.g., don't spawn on the player)
            bool tooClose = false;
            if (avoidObject != null)
            {
                if (Vector3.Distance(pos, avoidObject.transform.localPosition) < minDistance)
                {
                    tooClose = true;
                }
            }

            if (colliders.Length == 0 && !tooClose) return pos;
            attempts++;
        }
        
        // Fallback if no spot found
        return new Vector3(Random.Range(-2f, 2f), y, Random.Range(-2f, 2f));
    }

    public void ResetEnvironment()
    {
        // 1. Move Goal first
        goal.transform.localPosition = FindSafePos(trainingArea.transform.position, 0.74f);
    
        // 2. Move Player (far enough from Goal)
        player.transform.localPosition = FindSafePos(trainingArea.transform.position, 0.74f, goal, 6.0f);
    
        // 3. Move Agent (far enough from Player to prevent instant-win)
        transform.localPosition = FindSafePos(trainingArea.transform.position, 1.0f, player, 5.0f);

        // 4. Move Obstacles
        for (int i = 0; i < obsCount; i++)
        {
            obstacles[i].transform.localPosition = FindSafePos(trainingArea.transform.position, 0.5f);
        }
    }

    public override void OnEpisodeBegin()
    {
        episodeNum = stats.StartEpisode();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        var playerb = player.GetComponent<Rigidbody>();
        playerb.linearVelocity = Vector3.zero;
        playerb.angularVelocity = Vector3.zero;

        ResetEnvironment();
        lastDist = Vector3.Distance(player.transform.localPosition, transform.localPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (inHumanControl) return;

        // 1. Relative Velocity (2 floats)
        Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.z);

        // 2. Normalized Distance (1 float)
        float dist = Vector3.Distance(player.transform.localPosition, transform.localPosition);
        sensor.AddObservation(dist / 20f); 

        // 3. Direction to player (3 floats)
        Vector3 dirToPlayer = (player.transform.localPosition - transform.localPosition).normalized;
        sensor.AddObservation(dirToPlayer);
        
        // 4. Agent Forward Direction (3 floats)
        sensor.AddObservation(transform.forward);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        move = actions.ContinuousActions[0];
        turn = actions.ContinuousActions[1];

        CalcMoveReward();
    }

    private void FixedUpdate()
    {
        if (inHumanControl) return;

        // Apply Movement
        rb.AddForce(transform.forward * move * moveForce, ForceMode.Force);
        if (rb.linearVelocity.magnitude > MAX_MAGNITUDE)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * MAX_MAGNITUDE;
        }
        transform.Rotate(Vector3.up * turn * turnForce * Time.fixedDeltaTime);

        
    }

    void CalcMoveReward()
    {
        // Calculate Reward Logic
        Vector3 dirToPlayer = (player.transform.localPosition - transform.localPosition).normalized;
        
        // Dot product: +1 if moving directly toward player, -1 if moving away
        float velocityAlignment = Vector3.Dot(rb.linearVelocity.normalized, dirToPlayer);
        
        // Small incremental reward for moving the right way
        if(rb.linearVelocity.magnitude > 0.1f) 
        {
            //Debug.Log(velocityAlignment);
            if (velocityAlignment > 0) {
                AddReward(velocityAlignment * 0.002f);
            } else
            {
                //AddReward(velocityAlignment * 0.001f);
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            SetReward(goalReward);
            stats.AddGoal(episodeNum, 1);
            EndEpisode();
        }
        else if (collision.gameObject.CompareTag("Death"))
        {
            SetReward(deathReward);
            stats.AddGoal(episodeNum, 0);
            EndEpisode();
        }

    }

    public void PlayerGotGoal()
    {
        Debug.Log("<color=#0000ff>Player Reached Goal</color>");
        SetReward(failReward);
        stats.AddGoal(episodeNum, 0);
        EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        /*
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
        */
    }
}

