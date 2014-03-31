using UnityEngine;
using System.Collections;

public class Enemy : MonoBehaviour {

	private int destroyCount = -1;

	// Use this for initialization
	void Start () {
	
	}

	
	void OnCollisionEnter(Collision collision) {
		print ("Enemy COLLISION with " + collision.gameObject.name);
		renderer.material.color = Color.red;
		destroyCount = 20;
		if (collision.gameObject.name == "Brush") {
			rigidbody.velocity = (collision.gameObject.transform.position - transform.position) * 10.0f;
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (destroyCount-- == 0) {
			Destroy(gameObject);
		}
	}
}
