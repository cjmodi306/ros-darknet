pipeline {
    agent any
    stages {
        stage('Build') {
            steps {
  		sh 'echo "Starting..."'
	     }
	}
	stage('Test') {
            steps {
               dir('.'){
               sh 'echo "${env.WORKSPACE}"'
               }
		  }
        }
     }
}
