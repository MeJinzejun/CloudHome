var mqtt = require('../../utils/mqtt.min.js')
const crypto = require('../../utils/hex_hmac_sha1.js')
var client;
var SendMSG;


Page({
    
  data: {
    
    password_input:"",
    key:"123456",
    login: false,
    button_clicking: false,
    iot_connect: false,
    connect_text: "未连接",
  },

  inputPwd:function(e){
    this.setData({
      password_input: e.detail.value
    })
  },

  confirmPwd:function(){
    var pwd = this.data.password_input;
    var that = this
    if(pwd != this.data.key){
      wx.showToast({
        title: '密码错误',
        icon: 'none',
        duration: 2000
      })
    }else{
      wx.showToast({
        title: '验证通过',
        icon: 'success',
        duration: 2000
      })
      wx.setStorage({
        key: "password",
        data: pwd,
      })
      this.login()
    }
  },

  login:function(){
    var that = this
    wx.getStorage({
      key: 'password',
      success(res) {
        console.log(res)
        var pwd = res.data
        if (pwd == that.data.key) {
          that.setData({
            login: true
          })
          that.doConnect()
        }
      }
    })
  },

  onLoad: function () {
    this.login()
  },
  doConnect() {
    var that = this;
   // 自己的阿里云三元组
    const deviceConfig = {
      productKey: "jk1zFbWTW6V",
      deviceName: "rack-Console",
      deviceSecret: "1abcca753dfa457b96551bb5cb0af996",
      regionId: "cn-shanghai"
    };
    const options = this.initMqttOptions(deviceConfig);
    console.log(options)
    
    client = mqtt.connect('wxs://jk1zFbWTW6V.iot-as-mqtt.cn-shanghai.aliyuncs.com', options)
    client.on('connect', function () {
      that.setData({
        connect_text: "连接服务器成功",
        iot_connect: true
      })
///jk1zFbWTW6V/rack-Console/user/get
      //订阅主题，替换productKey和deviceName(这里的主题可能会不一样，具体请查看后台设备Topic列表或使用自定义主题)
       client.subscribe('/jk1zFbWTW6V/rack-Console/user/wxmessage', function (err) {
         if (!err) {
           console.log('订阅成功！');
         }
       })
    })
    //接收消息监听
    client.on('message', function (topic, message) {
      // message is Buffer
      console.log('收到消息：' + message.toString())
      //关闭连接 client.end()
      var data = JSON.parse(message.toString())// 将消息转换为JSON对象, 其消息一定得是JSON格式
      //console.log(data);
      if(data.items.temp){
        var tempstr = data.items.temp.value;
        //console.log(tempstr);
        that.setData({
            temperature: tempstr
        })
      }
      if(data.items.hum){
        var humstr = data.items.hum.value;
        //console.log(tempstr);
        that.setData({
            humidity: humstr
        })
      }
      if(data.items.guangmin){
        var guangminstr = data.items.guangmin.value;
        //白天=0,黑夜=1
        if(guangminstr){
            that.setData({
                GmStatusMSG : '黑夜'     
            })
        }else{
            that.setData({
                GmStatusMSG : '白天'   
            })
        }
      }
      if(data.items.renti){
        var rentistr = data.items.renti.value;
        //有人=1,无人=0
        if(rentistr){
            that.setData({
                RtStatusMSG : '有人'    
            })
        }else{
            that.setData({
                RtStatusMSG : '无人'   
            })
        }
      }
      if(data.items.yudi){
        var yudistr = data.items.yudi.value;
       //有雨=0
        if(yudistr){
            that.setData({
                YdStatusMSG : '无雨'    
            })
        }else{
            that.setData({
                YdStatusMSG : '有雨'   
            })
        }
      }
    })
  },
  //IoT平台mqtt连接参数初始化
  initMqttOptions(deviceConfig) {

    const params = {
      productKey: deviceConfig.productKey,
      deviceName: deviceConfig.deviceName,
      timestamp: Date.now(),
      clientId: Math.random().toString(36).substr(2),
    }
    //CONNECT参数
    const options = {
      keepalive: 60, //60s
      clean: true, //cleanSession不保持持久会话
      protocolVersion: 4 //MQTT v3.1.1
    }
    //1.生成clientId，username，password
    options.password = this.signHmacSha1(params, deviceConfig.deviceSecret);
    options.clientId = `${params.clientId}|securemode=2,signmethod=hmacsha1,timestamp=${params.timestamp}|`;
    options.username = `${params.deviceName}&${params.productKey}`;

    return options;
  },


  
  button_style(){
    var that = this
    this.setData({
      img_url: "/img/door_b.png",
      button_clicking: true
    })
    setTimeout(function(){
      that.setData({
        img_url: "/img/door_a.png",
        button_clicking: false
      })
    }, 2000)
  },
  onLedChange(event){
    const that = this
    console.log(event.detail.value)
    const sw=event.detail.value
    that.setData({Led:sw})
    if(sw){
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"7"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
            if (!err) {
              console.log('台灯开指令7发送成功！');
              that.setData({
                SendMSG : '台灯开指令7发送成功！'    
            })
            }
          })
    }else{
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"8"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
            if (!err) {
              console.log('台灯关指令8发送成功！');
              that.setData({
                SendMSG : '台灯关指令8发送成功！'    
            })
              
            }
          })
    }
  },

  MotorChange(event){
    const that = this
    console.log(event.detail.value)
    const sw=event.detail.value
    that.setData({Led:sw})
    if(sw){
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"1"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('窗帘拉出指令1发送成功！');
            that.setData({
                SendMSG : '窗帘拉出指令1发送成功！'    
            })
          }
        })
    }else{
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"2"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('窗帘收回指令2发送成功！');
            that.setData({
                SendMSG : '窗帘收回指令2发送成功！'    
            })
          }
        })
    }
  },
  DoorChange(event){
    const that = this
    console.log(event.detail.value)
    const sw=event.detail.value
    that.setData({Led:sw})
    if(sw){
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"3"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('舵机正转指令3发送成功！');
            that.setData({
                SendMSG : '舵机正转指令3发送成功！'    
            })
          }
        })
    }else{
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"4"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('舵机反转指令4发送成功！');
            that.setData({
                SendMSG : '舵机反转指令4发送成功！'    
            })
          }
        })
    }
  },
  FanChange(event){
    const that = this
    console.log(event.detail.value)
    const sw=event.detail.value
    that.setData({Led:sw})
    if(sw){
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"5"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('风扇开启指令5发送成功！');
            that.setData({
                SendMSG : '风扇开启指令5发送成功！'    
            })
          }
        })
    }else{
        var connectText = '{"method":"thing.event.property.set","TargetDevice":"class-rack","PowerSwitch_1":"6"}'
        client.publish('/jk1zFbWTW6V/rack-Console/user/wxmessage', connectText, function (err){
          if (!err) {
            console.log('风扇停止指令6发送成功！');
            that.setData({
                SendMSG : '风扇停止指令6发送成功！'    
            })
          }
        })
    }
  },


  
  /*
    生成基于HmacSha1的password
    参考文档：https://help.aliyun.com/document_detail/73742.html?#h2-url-1
  */
  signHmacSha1(params, deviceSecret) {

    let keys = Object.keys(params).sort();
    // 按字典序排序
    keys = keys.sort();
    const list = [];
    keys.map((key) => {
      list.push(`${key}${params[key]}`);
    });
    const contentStr = list.join('');
    return crypto.hex_hmac_sha1(deviceSecret, contentStr);
  }
  

  

})

