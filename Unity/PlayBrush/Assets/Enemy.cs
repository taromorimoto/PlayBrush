using UnityEngine;
using System.Collections;

public class Enemy : MonoBehaviour {

	private int destroyCount = -1;
	private GameObject brush;

	// Use this for initialization
	void Start () {
		brush = GameObject.Find("Brush");
	}

	
	void OnCollisionEnter(Collision collision) {
		print ("Enemy COLLISION with " + collision.gameObject.name);
		if (collision.gameObject.name == "Brush") {
			renderer.material.color = Color.red;
			destroyCount = 1;
			rigidbody.velocity = (collision.gameObject.transform.position - transform.position) * 10.0f;

			brush.GetComponent<AudioTest>().enemyDeath(this);
		}
	}
	
	// Update is called once per frame
	void Update () {

		rigidbody.velocity = (brush.transform.position - transform.position) * 0.3f;

		if (destroyCount-- == 0) {
			Destroy(gameObject);
		}
	}
}
