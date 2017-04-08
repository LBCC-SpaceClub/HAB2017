#!/usr/bin/env python
import requests
import json

class Ubiquity(object):
    ''' Methods to get information from the Ubiuity radio modem '''
    _host = 'http://192.168.1.20'
    _user = 'ubnt'
    _pass = 'ubnt'
    _loginPage = '/login.cgi'
    _statusPage = '/status.cgi'
    _session = None

    def __init__(self):
        ''' Prep the object by starting a new session and logging in '''
        self._session = requests.Session()
        self._session.get(self._host+self.loginPage)  # This line has magic sauce
        data={'uri': '','username': self._user,'password': self._pass}
        # Use files trick to post using multipart/form-data encoding.
        self._session.post(
            self._host+self.loginPage,
            files={k: (None, v) for k, v in data.items()},
            verify=False
        )

    def getRSSI(self):
        ''' Returns the RSSI value (0-100) '''
        pjson = json.loads(self._session.get(HOST+self._statusPage))
        return pjson.['RSSI']
