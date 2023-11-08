const TF_converter = require('./tf_converter/TF_converter.js');
const rosnodejs = require('rosnodejs');
const actionlib = rosnodejs.require('actionlib');
const express = require('express');
const app = express();
const cors = require('cors');
const port = 3000;
const bodyParser = require('body-parser');
app.use(cors()); // 这里添加CORS中间件
app.use(bodyParser.json());
app.use(express.json());



let currentPose = null;

// 初始化ROS節點
rosnodejs.initNode('/my_ros_node')
.then((rosNode) => {
   // ---------------subscribe-------------------//

   const nh = rosnodejs.nh;
   nh.subscribe('/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped', (msg) => {
      currentPose = msg.pose.pose; // 儲存當前位置
   });

   // ---------------subscribe-------------------//


   // ---------------publish---------------//

   const std_msgs = rosnodejs.require('std_msgs').msg;
   let initialize_Relocation_pub = nh.advertise('/initialize_Relocation', std_msgs.Int16 );
   let stopnavigation_pub = nh.advertise("/stopnavigationSend",std_msgs.Int8);

   const geometry_msgs = rosnodejs.require('geometry_msgs').msg;
   let goalPub = nh.advertise('/move_base_simple/goal', geometry_msgs.PoseStamped);

   // ---------------publish---------------//

   const client = new actionlib.ActionClient({
      nh,
      actionServer: '/move_base', 
      actionName: 'move_base_msgs/MoveBaseAction',
      actionMsgType: 'move_base_msgs/MoveBaseActionGoal'});

    // 中间件，允许跨域请求 (CORS)
    app.use((req, res, next) => {
    res.header('Access-Control-Allow-Origin', '*');
    res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
    next();
    });

   
   //-------------------------API-------------------------//
   // API current_pose
   app.get('/current_pose', (req, res) => {
      if (currentPose) {
        res.json(currentPose);
      } else {
        res.status(404).send('No pose information available.');
      }
   });

   //API Relocation_initpose
   app.post('/relocation_initpose', express.json(), (req, res) => {
   const value = req.body.relocation_initpose_value;
   console.log('Received value:', value);
   // check if a value was provided and if it is a number
   if (value !== undefined && !isNaN(value)) {
      let msg = new std_msgs.Int16();
      msg.data = value;
      initialize_Relocation_pub.publish(msg);
      res.status(200).send('Relocation_initpose Message published successfully.');
   } else {
      res.status(400).send('No valid number provided.');
   }
   });

   //API Path Cancel
   app.post('/path_cancel', express.json(), (req, res) => {
      const path_cancel_value = req.body.path_cancel_value;
      console.log('Received value:', path_cancel_value);
      // check if a path_cancel_value was provided and if it is a number
      if (path_cancel_value !== undefined && !isNaN(path_cancel_value)) {
         let msg = new std_msgs.Int8();
         msg.data = path_cancel_value;
         stopnavigation_pub.publish(msg);
         res.status(200).send('Path Cancel Message published successfully.');
      } else {
         res.status(400).send('No valid number provided.');
      }
   });

   //API set_goal
   app.post('/set_goal', express.json(), (req, res) => {
      // const x = req.body.x;
      // const y = req.body.y;
      // const yaw = req.body.yaw;  // 接收前端发送的偏航角度
      const multiple_points = req.body.multiple_points;
      console.log('Received value:', multiple_points);
      let goals = [];
    
      // 确保收到了所有必要的坐标和角度值
      if (multiple_points !== null) {
         
         for(let i=0;i<multiple_points.length;i++){
            // 新的PoseStamped消息
            let goalMsg = new geometry_msgs.PoseStamped();
         
            // 设置header
            goalMsg.header.stamp = rosnodejs.Time.now();
            goalMsg.header.frame_id = 'map'; // 或者是任何合适的参考框架
         
            // 设置位置
            goalMsg.pose.position.x = multiple_points[i].x;
            goalMsg.pose.position.y = multiple_points[i].y;
            goalMsg.pose.position.z = 0;  // 在2D导航中，通常Z值为0
         
            // 将偏航角度转换为四元数
            const quaternion = TF_converter(multiple_points[i].yaw, 0, 0);
         
            // 设置姿态
            goalMsg.pose.orientation.x = quaternion.x;
            goalMsg.pose.orientation.y = quaternion.y;
            goalMsg.pose.orientation.z = quaternion.z;
            goalMsg.pose.orientation.w = quaternion.w;

            goals.push(goalMsg);
         
            // 发布目标点
            // goalPub.publish(goalMsg);

            // res.status(200).send('Goal Arrived, moving to next goal.');
            // console.log('goalMsg.pose:', goalMsg.pose);
         }
         goals.forEach((goal, index) =>{
            client.sendGoal(goal);
            client.waitForResult();
            res.status(200).send('Goal Arrived, moving to next goal.');
         })

         res.status(200).send('Goal set successfully.');
      } else {
        res.status(400).send('Invalid parameters provided.');
      }
    });

   //API Go_home
   app.post('/go_home', express.json(), (req, res) => {
      const go_home_value = req.body.go_home_value;
      console.log('Received value:', go_home_value);
    
      // 确保收到了所有必要的坐标和角度值
      if (go_home_value!== undefined && go_home_value === 40) {
        // 新的PoseStamped消息
        let goalMsg = new geometry_msgs.PoseStamped();
    
        // 设置header
        goalMsg.header.stamp = rosnodejs.Time.now();
        goalMsg.header.frame_id = 'map'; // 或者是任何合适的参考框架
    
        // 设置位置
        goalMsg.pose.position.x = 0;
        goalMsg.pose.position.y = 0;
        goalMsg.pose.position.z = 0;  // 在2D导航中，通常Z值为0
    
        // 设置姿态
        goalMsg.pose.orientation.x = 0;
        goalMsg.pose.orientation.y = 0;
        goalMsg.pose.orientation.z = 0;
        goalMsg.pose.orientation.w = 1;
    
        // 发布目标点
        goalPub.publish(goalMsg);
        
        res.status(200).send('Go Home Msg Send successfully.');
      } else {
        res.status(400).send('Invalid parameters provided.');
      }
    });

    
    

   // 監聽指定端口
   app.listen(port, () => {
      console.log(`Server running on port:${port}`);
   });
})
.catch((error) => {
   console.error('ROS Node failed to initialize', error);
});
