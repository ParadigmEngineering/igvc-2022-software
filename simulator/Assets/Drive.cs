using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Drive : MonoBehaviour
{

    public Rigidbody rb;
    public float speed;
    public float torque;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
            resetScene();

        manualControls();
    }

    void manualControls()
    {
        if (Input.GetKey(KeyCode.W))
        {
            rb.AddRelativeForce(speed * Time.deltaTime, 0, 0);
        }
        else if (Input.GetKey(KeyCode.S))
        {
            rb.AddRelativeForce(-speed * Time.deltaTime, 0, 0); ;
        }

        if (Input.GetKey(KeyCode.A))
        {
            rb.AddRelativeForce(0, 0, speed * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.D))
        {
            rb.AddRelativeForce(0, 0, -speed * Time.deltaTime);   
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            rb.AddTorque(transform.up * torque * Time.deltaTime);
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            rb.AddTorque(-transform.up * torque * Time.deltaTime);
        }
    }

    public void resetScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name); // resets lvl
    }
}
