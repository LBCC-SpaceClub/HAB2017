#!/usr/bin/env python
import requests
import json

class Ubiquity(object):
    ''' Methods to get information from the Ubiuity radio modem '''
    host = 'http://192.168.1.20'
    user = 'ubnt'
    pass = 'ubnt'
    loginPage = '/login.cgi'
    statusPage = '/status.cgi'
    session = None

    def init(self):
        ''' Prep the object by starting a new session and logging in '''
        self.session = requests.Session()
        self.session.get(self.host+self.loginPage)  # This line has magic sauce
        data={'uri': '','username': self.user,'password': self.pass}
        # Use files trick to post using multipart/form-data encoding.
        self.session.post(
            self.host+self.loginPage,
            files={k: (None, v) for k, v in data.items()},
            verify=False
        )

    def getRSSI(self):
        ''' Returns the RSSI value (0-100) '''
        pjson = json.loads(self.session.get(HOST+self.statusPage))
        return pjson.['RSSI']
