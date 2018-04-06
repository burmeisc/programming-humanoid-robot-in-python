""" Abgewandelt nach: Say 'hello, you' each time a human face is detected
    Cannot be tested on simulated robot
"""

import sys
import time

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "nao.local"


# Global variable to store the HumanGreeter module instance
ColourOrder = None
memory = None


class SpeechDetection(ALModule):
    """ A simple module able to react
    to speech events

    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")

        # Subscribe to the WordRecognized event:
        global memory
        memory = ALProxy("ALMemory")
        speech = ALProxy("ALSpeechRecognition")
        # List of colours which can be recognized
        speech.setVocabulary(["red","blue","black"])
        memory.subscribeToEvent("WordRecognized",
            "ColourOrder",
            "onColourDetected")

    def onColourDetected(self, *_args):
        """ This will be called each time a colour order (spoken) is
        detected.

        """
        # Unsubscribe to the event when reacting,
        # to avoid repetitions
        memory.unsubscribeToEvent("WordRecognized",
            "ColourOrder")

        self.tts.say("Okay, I heard you.")
        #TO DO: next step: vision module --> find colour/point to colour

        # Subscribe again to the event
        memory.subscribeToEvent("WordRecognized",
            "ColourOrder",
            "onColourDetected")


def main():
    """ Main entry point

    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       pip,         # parent broker IP
       pport)       # parent broker port


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable
    global ColourOrder
    ColourOrder = ColourOrder("ColourOrder")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)



if __name__ == "__main__":
    main()