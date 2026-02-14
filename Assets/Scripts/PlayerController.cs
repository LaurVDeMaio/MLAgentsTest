using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    Rigidbody rb;
    GameObject goal, trainingArea, environment;
    public float speed = 5.0f;
    AgentController agentController;
    Stats stats;

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        trainingArea = transform.parent.gameObject;
        if (trainingArea.name != "TrainingArea") trainingArea = trainingArea.transform.parent.gameObject;

        environment = trainingArea.transform.Find("Environment").gameObject;
        goal = trainingArea.transform.Find("Goal").gameObject;

        agentController = trainingArea.transform.Find("Agent").gameObject.GetComponent<AgentController>();

        stats = GameObject.FindGameObjectWithTag("Stats").GetComponent<Stats>();
    }


    void FixedUpdate()
    {

        if (goal != null)
        {
            float step = speed * Time.fixedDeltaTime;

            Vector3 targetDir = goal.transform.position - transform.position;
            Vector3 newDir = Vector3.RotateTowards(transform.forward, targetDir, step, 0.0f);
            transform.rotation = Quaternion.LookRotation(newDir);

            targetDir.Normalize();
            rb.MovePosition(rb.position + targetDir * step);
        }

    }
    
    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Goal"))
        {
            agentController.PlayerGotGoal();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Goal"))
        {
            agentController.PlayerGotGoal();
        }
    }
}
