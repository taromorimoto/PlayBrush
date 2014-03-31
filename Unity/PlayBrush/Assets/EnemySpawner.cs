using UnityEngine;
using System.Collections;

public class EnemySpawner : MonoBehaviour {

	public int maxEnemyCount = 5;
	public int enemyCount = 0;

	// Use this for initialization
	void Start () {

	
	}
	
	// Update is called once per frame
	void Update () {
		GameObject[] gos = GameObject.FindGameObjectsWithTag("Respawn");

		if (gos.Length < maxEnemyCount) {
			GameObject enemy = (GameObject)Instantiate(Resources.Load("Enemy"));
			Vector3 pos = new Vector3(Random.Range(-1, 1), Random.Range(-1, 1), Random.Range(-0.5f, 0.5f));
			enemy.transform.position = pos.normalized * 3;
		}
	
	}
}
