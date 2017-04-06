#!/usr/bin/env python
# -*_ coding:utf-8 _*_

'''
AIUISerial.py:  Handling serial messages of AIUI,
                including receiving and parsing data,
                and sending ctrl msgs.
'''

import json
import serial
import threading
import time
import array
import gzip
from cStringIO import StringIO
import pprint
import traceback

__author__ = 'Yuxiang Gao'
__copyright__ = 'Copyright (C) 2017 Yuxiang Gao'
__email__ = 'gaoyuxiang@stu.xjtu.edu.cn'
__license__ = 'GPL'
__version__ = '1.1'

AIUIAppid = '583c10e6'
AIUIKey = '2d8c2fa8a465b0dcbaca063e9493a2d9'
AIUIScene = 'main'
TTSText = '黑化肥会发黑'


class MyPrettyPrinter(pprint.PrettyPrinter):
    def format(self, object, context, maxlevels, level):
        if isinstance(object, unicode):
            return (object.encode('utf8'), True, False)
        return pprint.PrettyPrinter.format(self, object,
                                           context, maxlevels, level)


def json_load_byteified(file_handle):
    return _byteify(
        json.load(file_handle, object_hook=_byteify),
        ignore_dicts=True
    )


def json_loads_byteified(json_text):
    return _byteify(
        json.loads(json_text, object_hook=_byteify),
        ignore_dicts=True
    )


def _byteify(data, ignore_dicts=False):
    # if this is a unicode string, return its string representation
    if isinstance(data, unicode):
        return data.encode('utf-8')
    # if this is a list of values, return list of byteified values
    if isinstance(data, list):
        return [_byteify(item, ignore_dicts=True) for item in data]
    # if this is a dictionary, return dictionary of byteified keys and values
    # but only if we haven't already byteified it
    if isinstance(data, dict) and not ignore_dicts:
        return {
            _byteify(key,
                     ignore_dicts=True): _byteify(value, ignore_dicts=True)
            for key, value in data.iteritems()
        }
    # if it's anything else, return it in its original form
    return data


class tts_msg(object):
    def __init__(self, action, text):
        self.action = action
        if self.action == 'start':
            self.text = text

    def construct(self):
        if self.action == 'start':
            ttsMsg = {'type': 'tts',
                      'content': {'action': 'start', 'text': self.text}}
        else:
            ttsMsg = {'type': 'tts', 'content': {'action': 'stop'}}
        ttsJsonMsg = json.dumps(ttsMsg)
        # ttsJsonMsg = '{"type": "tts", "content": {"action": "start", "text": "不吃葡萄就吐葡萄皮"} }'
        print 'constructing tts msg'
        print ttsJsonMsg
        return ttsJsonMsg


class aiui_msg_handler(object):  # 0x04
    # Handler for AIUI messages
    def __init__(self, jsonMsg):
        self.msgType = jsonMsg['type']
        # aiui_event: Result Error State Wakeup Sleep Vad;
        content = jsonMsg['content']
        if self.msgType == 'aiui_event':
            # print content.keys()
            eventType = content.get('eventType', None)
            arg1 = content.get('arg1', None)
            arg2 = content['arg2']
            info = content.get('info', None)
            result = content.get('result', None)
            if eventType == 1:  # EVENT_RESULT
                self.eventType = "eventResult"
                # self.info = self.content['info']
                # self.result = self.content['result']
                self.sub = info['data'][0]['params']['sub']
                if self.sub == 'iat':
                    # print jsonMsg
                    self.eventType = "eventResultIAT"
                    self.recogResult = [ws['cw'][0]['w']
                                        for ws in result['text']['ws']]
                # elif self.sub is 'nlp':
            elif eventType == 2:  # EVENT_ERROR
                self.eventType = "eventError"
                self.errorCode = arg1
                self.errorMsg = info
            elif eventType == 3:  # EVENT_STATE
                self.eventType = "eventState"
                self.state = arg1
            elif eventType == 4:  # EVENT_WAKEUP
                self.eventType = "eventWakeup"
                self.info = content['info']
                self.angle = info['angle']
                self.beam = info['beam']
            elif eventType == 5:  # EVENT_SLEEP
                self.eventType = "eventSleep"
            elif eventType == 6:  # EVENT_VAD
                self.eventType = "eventVAD"
            elif eventType == 8:  # EVENT_CMD_RETURN
                self.eventType = "eventCmdReturn"
                self.cmdType = arg1
                self.isSuccess = (arg2 == 0)
                self.cmdInfo = info
            else:
                print 'Wrong input for aiui_event'
        elif self.msgType is 'wifi_status':
            self.wifiConnection = content['connected']
            self.ssid = content['ssid']
        elif self.msgType == 'tts_event':
            print 'tts_event'
            self.ttsStart = (True if (content['eventType'] is 0) else False)
            self.ttsError = content.get('error', None)
        else:
            print 'Not an aiui_event'

    def get_msg_type(self):
        # get event type
        try:
            return self.msgType
        except:
            print 'msg type error for aiui_event'

    def get_type(self):
        # get event type
        try:
            return self.eventType
        except:
            print 'type error for aiui_event'

    def get_angle(self):
        # get wakeup angle
        try:
            return self.angle
        except:
            print 'no angle for aiui_event'

    def get_beam(self):
        # get wakeup beam
        try:
            return self.beam
        except:
            print 'no beam for aiui_event'

    def get_result(self):
        # get iat result
        try:
            # print 'iat result~: '
            # for ws in self.recogResult:
            #     print ws
            return self.recogResult
        except:
            print 'no result for aiui_event'

    def get_error(self):
        # get error
        try:
            print ('error code : {0}, '
                   'error message : {1}').format(self.errorCode, self.errorMsg)
        except:
            print 'no error for aiui_event'

    def get_state(self):
        # get current state: STATE_IDLE, STATE_READY, STATE_WORKING
        try:
            return self.state
            print 'AIUI state: ' + self.state
        except:
            print 'no result for aiui_event'

    def get_tts_state(self):
        try:
            if self.ttsError is not None:
                print self.ttsError
            return self.ttsStart
        except:
            print 'no state for tts_event'


class aiui_ctrl_msg(object):  # 0x05
    # Constructor for AIUI control messages
    def __init__(self, aiuiCtrlType, **content):
        self.aiuiType = aiuiCtrlType
        if aiuiCtrlType == 'aiui_msg':
            # aiui_ctrl_msg('aiui_msg', msg_type='reset')
            self.msgContent = {'msg_type': 0,
                               'arg1': 0,
                               'arg2': 0,
                               'params': ""}
            if content.get('msg_type', None) == 'state':
                self.msgContent['msg_type'] = 1
            elif content.get('msg_type', None) == 'reset':
                self.msgContent['msg_type'] = 4
            elif content.get('msg_type', None) == 'start':
                self.msgContent['msg_type'] = 5
            elif content.get('msg_type', None) == 'stop':
                self.msgContent['msg_type'] = 6
            elif content.get('msg_type', None) == 'reset_wakeup':
                self.msgContent['msg_type'] = 8
            elif content.get('msg_type', None) == 'set_beam':
                self.msgContent['msg_type'] = 9
                self.msgContent['arg1'] = content['arg1']
            elif content.get('msg_type', None) == 'set_params':
                self.msgContent['msg_type'] = 10
                self.msgContent['params'] = content['params']
            elif content.get('msg_type', None) == 'upload_lexicon':
                self.msgContent['msg_type'] = 11
                self.msgContent['params'] = content['params']
            elif content.get('msg_type', None) == 'send_log':
                self.msgContent['msg_type'] = 12
                self.msgContent['params'] = content['params']
            elif content.get('msg_type', None) == 'build_grammer':
                self.msgContent['msg_type'] = 16
                self.msgContent['params'] = content['params']
            elif content.get('msg_type', None) == 'update_local_lexicon':
                self.msgContent['msg_type'] = 17
                self.msgContent['params'] = content['params']
            else:
                print 'Wrong input for aiui_msg'
        elif aiuiCtrlType == 'voice':
            # aiui_ctrl_msg('aiui_msg', enable_voice=True)
            self.msgContent = {'enable_voice':
                               (False if (content.get('enable_voice', None)
                                is False) else True)}
        elif aiuiCtrlType == 'status':
            # aiui_ctrl_msg('status')
            self.msgContent = {'query': 'wifi'}
        elif aiuiCtrlType == 'save_audio':
            # aiui_ctrl_msg('save_audio', save_len=10)
            self.msgContent = {'save_len': content.get('save_len', 0)}
        elif aiuiCtrlType == 'tts':
            # aiui_ctrl_msg('tts', action='start', text='')
            ttsMsg = tts_msg(content['action'], content.get('text', None))
            self.msgContent = ttsMsg.construct()
        # elif aiuiCtrlType is 'handshake':
        #     self.msgContent = [165, 0, 0, 0]
        elif aiuiCtrlType == 'aiui_cfg':
            # aiui_ctrl_msg('aiui_cfg')
            self.msgContent = {'appid': AIUIAppid,
                               'key': AIUIKey,
                               'scene': AIUIScene,
                               'launch_demo': True}
        else:
            print 'ctrl_msg input error'

    def construct(self):
        try:
            ctrlMsg = {'type': self.aiuiType,
                       'content': self.msgContent}
            return json.dumps(ctrlMsg)
        except:
            pass

    def construct_hex(self, msg_ID):
        t = array.array('B', [0xA5, 0x01,  # msg head & user ID
                              0x05,        # msg type
                              0x00, 0x00,  # msg length
                              0x00, 0x00   # msg ID
                              ])
        if self.aiuiType == 'aiui_cfg':
            t[2] = 0x03
        msg_ID += 1
        if msg_ID > 65535:
            msg_ID = 1
        t[5] = msg_ID & 0xff
        t[6] = (msg_ID >> 8) & 0xff
        # if self.type is 'handshake':
        #     t[2] = 0xff
        #     for d in self.construct():
        #         t.append(hex(d))
        # else:
        for ch in self.construct():
            t.append(ord(ch))
        msgLen = len(t) - 7
        t[3] = msgLen % 255
        t[4] = msgLen / 255
        t.append((~sum(t) + 1) & 0xff)
        print t
        return t


class COMThread(threading.Thread):
    def __init__(self, name='COMThread'):  # 初始化线程
        threading.Thread.__init__(self, name=name)
        self._stopevent = threading.Event()
        self._stopevent.clear()
        self.globalID = 0
        self.sendCnt = 0
        self.ackID = 0
        self.handshakeID = 0
        self.handshakeCnt = 0
        self.sendID = 0
        # 将串口定义为”/dev/xunfei，比特率为115200，超时0.4秒“
        self.ser = serial.Serial(port='/dev/tty.usbserial',
                                 baudrate=115200,
                                 timeout=0.5)
        if self.ser is None:
            print '无法打开串口'
        # else:
        #     aiuiCM = aiui_ctrl_msg('aiui_msg', msg_type='reset')
        #     aiuiCM.construct_hex(None, self.globalID)

    def flagget_len(self, str):
        return ord(str[3]) + ((ord(str[4])) << 8)

    def flagget_id(self, str):
        return ord(str[5]) + ((ord(str[6])) << 8)

    def send_ok(self, str, msgflag):
        print 'try handshake'
        if self.handshakeCnt > 20:
            self.stop()
        else:
            self.handshakeCnt += 1
            # TODO: Merge send_ok with aiui_ctrl_msg
            # acm = aiui_ctrl_msg('handshake')
            # self.ser.write(acm.construct_hex(self.globalID))
            t = array.array('B', [0xA5, 0x01,   # msg head & user ID
                                  msgflag,      # msg type
                                  0x04, 0x00,   # msg length
                                  0x00, 0x00,   # msg ID
                                  0xA5, 0x00,   # handshaking msg
                                  0x00, 0x00,   # handshaking msg
                                  0x00])        # checksum
            if str is None:
                self.globalID += 1
                if self.globalID > 65535:
                    self.globalID = 1
                t[5] = self.globalID & 0xff
                t[6] = (self.globalID >> 8) & 0xff
            else:
                t[5] = ord(str[5])
                t[6] = ord(str[6])
                self.globalID = t[5] + 255 * t[6]
            t[11] = (~sum(t) + 1) & 0xff
            print t
            self.ser.write(t)

    def send_msg(self, hexMsg):
        if self.sendCnt == 0:
            self.unsentMsg = hexMsg
            self.sendID = self.flaget_id(hexMsg)
            self.ser.write(hexMsg)
            self.sendCnt += 1
            self.sendSuccess = None
        elif self.sendCnt < 10:
            if self.sendID == self.msgID:
                self.sendCnt = 0
                self.sendSuccess = True
            else:
                self.sendSuccess = False
                self.ser.write(self.unsentMsg)
                self.sendCnt += 1
        else:
            print 'msg sending failed'

    def send_tts(self, cmd, ttstxt):
        # TODO: Merge send_tts with aiui_ctrl_msg
        # Done!
        print 'making tts msg'
        acm = aiui_ctrl_msg('tts', action=cmd, text=ttstxt)
        print self.globalID
        # self.ser.write(acm.construct_hex(self.globalID))
        self.send_msg(acm.construct_hex(self.globalID))
        print 'tts msg sent'
        # t = array.array('B', [0xA5, 0x01,  # msg head & user ID
        #                       0x05,        # msg type
        #                       0x00, 0x00,  # msg length
        #                       0x00, 0x00   # msg ID
        #                       ])
        # self.globalID += 1
        # if self.globalID > 65535:
        #     self.globalID = 1
        # t[5] = self.globalID & 0xff
        # t[6] = (self.globalID >> 8) & 0xff
        # ttsMsg = tts_msg(cmd,
        #                  '后果花园自从产品导致图文河南打印促进业务微微那，'
        #                  '只需导演海外总体静静别人图书馆女士宽带参与详细内容坚持男, '
        #                  '多次队员诱惑画面武器今年小学让人站在家庭反而硬盘下，'
        #                  '一场办公角色证据最佳个性.')
        # for ch in ttsMsg.construct():
        #     t.append(hex(ord(ch)))
        # msgLen = len(t) - 7
        # t[3] = hex(msgLen % 255)
        # t[4] = hex(msgLen / 255)
        # t.append((~sum(t) + 1) & 0xff)
        # self.ser.write(t)

    def parse_msg(self, flag, data):
        print 'getflag=%d' % ord(flag[2])
        if ord(flag[2]) == 0x4:  # AIUI message
            buf = StringIO(data)
            f = gzip.GzipFile(mode="rb", fileobj=buf)
            # loaded_json = json.loads(f.read())
            loadedJson = json_loads_byteified(f.read())
            # print loadedJson.keys()
            aiuiMsg = aiui_msg_handler(loadedJson)
            # json_msg = json.dumps(loaded_json,
            #                       ensure_ascii=False,
            #                       sort_keys=True,
            #                       indent=4,
            #                       separators=(',', ': '))
            # print f.read()
            # print loaded_json
            # print json_msg
            # pprint(loaded_json)
            MyPrettyPrinter().pprint(loadedJson)
            if aiuiMsg.get_msg_type() == 'aiui_event':
                if aiuiMsg.get_type() == 'eventWakeup':
                    print 'wakeup angle: ' + str(aiuiMsg.get_angle())
                elif aiuiMsg.get_type() == 'eventResultIAT':
                    print ('IAT result: ' + ' '.join(aiuiMsg.get_result()))
            elif aiuiMsg.get_msg_type() == 'tts_event':
                if aiuiMsg.get_tts_state():
                    print 'tts start'
                elif not aiuiMsg.get_tts_state():
                    print 'tts end'
                else:
                    print 'tts state err'
            return aiuiMsg
        else:
            return None

    def run(self):
        if self.ser is None:
            return
        print "%s starts" % (self.getName())
        while not self._stopevent.isSet():
            try:
                flag_read = self.ser.read()
                if(len(flag_read) > 0):
                    flag_read += self.ser.read(6)
                    if ((ord(flag_read[0]) == 165) and
                       (ord(flag_read[1]) == 1)):
                        # Recv data, flag is A5 01
                        dataLen = self.flagget_len(flag_read)
                        dataType = ord(flag_read[2])
                        self.msgID = self.flagget_id(flag_read)
                        print 'type:' + str(dataType)
                        print 'Len:' + str(dataLen)
                        print 'ID:' + str(self.msgID)
                        if (dataType == 1):  # handshaking message
                            self.send_ok(flag_read, 0xff)
                        elif (dataType == 255):  # confirmation message
                            self.handshakeID = self.msgID
                            self.handshakeCnt = 0
                        elif (dataType == 4 and
                              dataLen > 0 and
                              dataLen < 1024 * 1024):
                            data_read = self.ser.read(dataLen)
                            self.ser.read()   # checkdata ,just read and pass
                            parsedMsg = self.parse_msg(flag_read, data_read)
                            # aiui config start
                            # cfg = aiui_ctrl_msg('aiui_cfg')
                            # cfgHex = cfg.construct_hex(self.globalID)
                            # self.ser.write(cfgHex)
                            # aiui config end
                            if ((parsedMsg is not None) and
                               (parsedMsg.get_result() is not None)):
                                if ('葡萄' in ''.join(parsedMsg.get_result())):
                                    print 'sending tts'
                                    self.send_tts('start', TTSText)
                            print 'get one msg len=%d' % dataLen
            except serial.SerialException:
                print 'serial.SerialException ERROR'
                print traceback.format_exc()
                self.ser.close()
                print 'serial closed'
                continue
        if self.ser.isOpen():
            self.ser.close()  # 串口关闭
        print "%s ends" % (self.getName())

    # def my_tts(self):
    #     print 'sending tts'
    #     self.send_tts('start', TTSText)

    # 关闭串口线程
    def stop(self, timeout=None):
        if self.ser.isOpen():
            self.ser.close()
        self._stopevent.set()
        print "COMThread.join()"
        threading.Thread.join(self, timeout)


if __name__ == "__main__":
    th1 = COMThread()
    try:
        th1.start()
        # time.sleep(5)
        # th1.my_tts()
        time.sleep(20)
        th1.stop()
    except Exception, e:
        print e

    # if th1._stopevent.isSet():
    #     th1.stop()
