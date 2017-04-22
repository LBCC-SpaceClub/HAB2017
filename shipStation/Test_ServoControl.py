import unittest
import ServoControl

class Test_ServoControl(unittest.TestCase):

    def test_bearing(self):
        aLat = 44.564939
        aLon = -123.241243
        bLat = 44.565973
        bLon = -123.239418
        new_bearing = ServoControl.bearing(aLat, aLon, bLat, bLon)
        old_bearing = ServoControl.original_bearing(aLat, aLon, bLat, bLon)
        self.assertEqual(new_bearing, old_bearing)

if __name__ == '__main__':
    unittest.main()
