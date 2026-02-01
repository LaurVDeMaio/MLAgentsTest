using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class AgentController : Agent
{
    Rigidbody rb;

    //private BufferSensorComponent bufferSensor;

    [SerializeField]
    public bool inHumanControl;

    GameObject goal, trainingArea, environment, player;
    GameObject[] obstacles;
    public int obsCount = 1;
    public GameObject obstacle;

    Vector3 startingPosition;
    float lastDist;

    float move = 0;
    float turn = 0;

    Stats stats;

    private float moveForce = 2.0f;
    private float turnForce = 10.0f;

    private float deathReward = -0.5f;
    public float failReward = -0.75f;
    private float goalReward = 1.5f;
    private float rCloser = 0.25f;
    private float rFurther = -0.45f;

    // RaycastHit hit;
    // LayerMask collisions;
    
    public int episodeNum = 0;

    public override void Initialize()
    {
        //bufferSensor = GetComponent<BufferSensorComponent>();
    }

    void Start()
    {
        startingPosition = transform.localPosition;
        rb = GetComponent<Rigidbody>();

        stats = GameObject.FindGameObjectWithTag("Stats").GetComponent<Stats>();

        trainingArea = transform.parent.gameObject;
        if (trainingArea.name != "TrainingArea") trainingArea = trainingArea.transform.parent.gameObject;

        environment = trainingArea.transform.Find("Environment").gameObject;
        player = trainingArea.transform.Find("Player").gameObject;
        goal = trainingArea.transform.Find("Goal").gameObject;



        obstacles = new GameObject[obsCount];
        for (int i = 0; i < obsCount; i++)
        {
            var pos = new Vector3(0, 10000, 0);
            obstacles[i] = Instantiate(obstacle, pos, Quaternion.identity, environment.transform);
        }

    }
    public Vector3 FindSafePos(Vector3 offset, float y)
    {
        while (true)
        {
            Vector3 pos = new Vector3(Random.Range(-9.0f, 9.0f), 0.05f, Random.Range(-9.0f, 9.0f));
            var colliders = Physics.OverlapBox(pos + offset, new Vector3(3.0f, 0.25f, 3.0f));
            if (colliders.Length == 0) return new Vector3(pos.x,y, pos.z);
        }
    }



    public void RandomizeGoals()
    {
        if (goal != null) goal.transform.localPosition = FindSafePos(trainingArea.transform.position, 0.74f);
        
    }

     public void RandomizePlayer()
    {
        if (player != null) player.transform.localPosition = FindSafePos(trainingArea.transform.position, 0.74f);
        
    }

    public void RandomizeAgent()
    {
        if (rb != null) rb.transform.localPosition = FindSafePos(trainingArea.transform.position, 1.0f);

    }


    public void RandomizeObstaclesPosition()
    {
        //randomize position of each obstacle

        for(int i = 0; i < obsCount; i++)
        {
            obstacles[i].transform.localPosition = FindSafePos(trainingArea.transform.position, 0f);
        }
    }


   
    public override void OnEpisodeBegin()
    {
        episodeNum = stats.StartEpisode();

        transform.localPosition = startingPosition;
        lastDist = Vector3.Distance(player.transform.localPosition, transform.localPosition);
        var dir = player.transform.localPosition - transform.localPosition;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        RandomizeGoals();
        RandomizePlayer();
        RandomizeAgent();

        //instantiate a random number of obstacles
        RandomizeObstaclesPosition();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (inHumanControl) return;

        // Define half-extents based on your 20x1x20 floor
        float xLimit = 10f; 
        float zLimit = 10f;
        float yLimit = 4f; // Assuming 2 is the max height the agent might reach

        // 1. Normalize Agent Position
        // If x is 10, it becomes 1.0. If x is -10, it becomes -1.0.
        Vector3 agentPos = transform.localPosition;
        sensor.AddObservation(agentPos.x / xLimit);
        sensor.AddObservation(agentPos.y / yLimit);
        sensor.AddObservation(agentPos.z / zLimit);

        // List<GameObject> playerNobstacles = GetNearbyEntities(); 

        // foreach (var entity in playerNobstacles)
        // {
        //     Vector3 entityPos = entity.transform.localPosition;
        
        //     // 2. Normalize Entity Data
        //     float normX = entityPos.x / xLimit;
        //     float normY = entityPos.y / yLimit;
        //     float normZ = entityPos.z / zLimit;

        //     // One-Hot tags (Obstacle vs Player)
        //     float isObstacle = entity.CompareTag("Death") ? 1f : 0f;
        //     float isPlayer = entity.CompareTag("Player") ? 1f : 0f;

        //      // Buffer Observation (Size 5)
        //      float[] envData = new float[] { normX, normY, normZ, isObstacle, isPlayer };
        //      bufferSensor.AppendObservation(envData);
        //}

    // Distance to player (1 float)
    var dist = Vector3.Distance(player.transform.localPosition, transform.localPosition);
    sensor.AddObservation(dist);

    // // Velocity of the agent (3 floats)
    // // agents momentum
    // sensor.AddObservation(rb.linearVelocity);

    // // Direction to the player (3 floats)
    // // "which way" to turn
    // Vector3 dirToPlayer = (player.transform.localPosition - transform.localPosition).normalized;
    // sensor.AddObservation(dirToPlayer);
}

    // List<GameObject> GetNearbyEntities()
    // {
    //     List<GameObject> nearbyEntities = new List<GameObject>();
    //     float detectionRadius = 15f; // Define how far the agent can "see"

    //     Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius);
    //     foreach (var hitCollider in hitColliders)
    //     {
    //         if (hitCollider.gameObject.CompareTag("Player") || hitCollider.gameObject.CompareTag("Death"))
    //         {
    //             nearbyEntities.Add(hitCollider.gameObject);
    //         }
    //     }

    //     return nearbyEntities;
    // }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (inHumanControl) return;

        move = actions.ContinuousActions[0];
        turn = actions.ContinuousActions[1];

    }

    void DoingTheThing()
    {
        rb.MovePosition(rb.position + (transform.forward * move * moveForce * Time.fixedDeltaTime));
        transform.Rotate(0, turn * turnForce * Time.fixedDeltaTime, 0);

        var curdist = Vector3.Distance(player.transform.localPosition, transform.localPosition);
        if (curdist < lastDist)
        {
            SetReward(rCloser);
        }
        else
        {
            SetReward(rFurther);
        }

        lastDist = curdist;
    }


    void Update() {
        
    }

    void FixedUpdate() {

        DoingTheThing();

    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            Debug.Log("<color=#00ff00>Player Caught</color>");
            SetReward(goalReward);
            EndEpisode();

            stats.AddGoal(episodeNum, 1);
        }

        else if (collision.gameObject.CompareTag("Death"))
        {
            Debug.Log("<color=#ff0000>Try Again</color>");
            SetReward(deathReward);
            EndEpisode();

            stats.AddGoal(episodeNum, 0);
        }

        else if (collision.gameObject.CompareTag("Goal"))
        {
            Debug.Log("<color=#ff0000>Try Again</color>");
            SetReward(failReward);
            EndEpisode();

            stats.AddGoal(episodeNum, 0);
        }
        

    }

}
