using UnityEngine;
using System.Collections;

public class FinishScript : MonoBehaviour {

    public GameObject victorySign;

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}

    void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "Player")
        {
            Destroy(other.gameObject);
            victorySign.SetActive(true);
        }
    }
}
