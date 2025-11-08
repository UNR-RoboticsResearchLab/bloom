pipeline {
    agent any

    stages {
        stage('Build') {
            when {
                branch 'main'
            }
            steps { 
                sh 'echo "Building prod..."'
                sh './build.sh --prod'  // your build command
            }
        }
        stage('Build Dev') {
            when {
                branch 'development'
            }
            steps {
                sh 'echo "Building dev..."'
                sh './build.sh'
            }
        }
        stage('Test') {
            steps {
                sh './test.sh'
            }
        }
        stage('Deploy') {
            when {
                branch 'main'
            }
            steps {
                sh 'echo "Deploying main..."'
                sh './deploy.sh --prod'
            }
        }
        stage('Deploy Dev') {
            when {
                branch 'development'
            }
            steps {
                sh './deploy.sh'
            }
        }
    }
}
