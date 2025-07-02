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

        // joystick values
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // dragging data
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

        // 3D stuff
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
                this.connected = true
                this.loading = false
                this.setup3DViewer()
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
        sendCommand: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: 0.2, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            topic.publish(message)
        },

        // Joystick methods
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
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
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },

        // Robot display methods
        setup3DViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 200,
                height: 150,
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