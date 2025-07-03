let vueApp = new Vue({
    el: '#vueApp',
    data: {
        // ros connection
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: '',
        port: '9090',
        menu_title: 'ROS Connection',

        // joystick
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // dragging
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '50px',
            height: '50px',
        },
        // publisher
        pubInterval: null,
        // subscriber
        pose: { x: 0.0, y: 0.0, yaw: 0.0},

        // 3D visualization
        viewer: null,
        tfClient: null,
        urdfClient: null,
    },
    methods: {
        // ROS2 connection
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                // Subscribe to odom
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/fastbot_1/odom',
                    messageType: 'nav_msgs/msg/Odometry'
                })
                topic.subscribe((message) => {
                    this.pose.x = message.pose.pose.position.x
                    this.pose.y = message.pose.pose.position.y
                    const q = message.pose.pose.orientation
                    this.pose.yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    console.log(message)
                })
                this.connected = true
                this.loading = false
                this.setup3DViewer()
                this.setCamera()

                // Setup the map client and viewer
                this.mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 270,
                    height: 200
                })
                this.mapGridClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    continuous: true,
                })
                // Scale the canvas to fit to the map
                this.mapGridClient.on('change', () => {
                    this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                    this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y)
                })
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                this.unset3DViewer()
            })
        },
        disconnect: function() {
            this.ros.close()
        },

        // Twist msg publisher
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },

        // Goal pose publisher
        sendGoal: function(goalName) {
            // Define poses for each waypoint
            const waypoints = {
                'sofa':       { x: -2.5844, y: -1.1867, quaternion: { x: 0, y: 0, z: 0, w: 1. } },
                'kitchen':    { x: 0.8044, y: 2.8236, quaternion: { x: 0, y: 0, z: 1., w: 0 } },
                'living-room':{ x: 1.4296, y: -1.1812, quaternion: { x: 0, y: 0, z: 1., w: 0 } }
            }
            const goal = waypoints[goalName] || waypoints['sofa']

            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/goal_pose',
                messageType: 'geometry_msgs/msg/PoseStamped'
            })
            let message = new ROSLIB.Message({
                header: {
                    frame_id: "map",
                    stamp: { secs: 0, nsecs: 0 }
                },
                pose: {
                    position: {
                        x: goal.x,
                        y: goal.y,
                        z: 0.0
                    },
                    orientation: goal.quaternion
                }
            })
            topic.publish(message)
        },

        // Joystick methods
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
            // Start publishing at 10Hz
            if (!this.pubInterval) {
                this.pubInterval = setInterval(() => {
                    this.publish()
                }, 100)
            }
            // Listen for mouseup 
            document.addEventListener('mouseup', this.stopDrag)
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()

            // Stop publishing
            if (this.pubInterval) {
                clearInterval(this.pubInterval)
                this.pubInterval = null
            }
            // Remove mouseup listener and send stop
            document.removeEventListener('mouseup', this.stopDrag)
            this.publish()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = (50 - this.y) / 50;
            this.joystick.horizontal = -(this.x - 50) / 50;
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        
        // Camera method
        setCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 270,
                height: 200,
                topic: '/fastbot_1/camera/image_raw',
                ssl: true,
            })
        },

        // Robot display methods
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 300,
                height: 200,
                antialias: true,
                fixedFrame: 'fastbot_1_base_link'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: 'fastbot_1_base_link'
            })

            // Setup the URDF client
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_1_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },
    },
})