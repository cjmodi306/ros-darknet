pipeline {
    agent any
    stages {
        stage('Build') {
            steps {
                sh 'ls'
                sh 'cd ..'
                sh 'ls'
	     }
	}
	stage('Test') {
            steps {
               dir('.'){sh 'ls'}
		  }
        }
     }
}
