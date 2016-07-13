#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import sys,  os
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import rospkg
from std_msgs.msg import String

class art_speech():
    
    def __init__(self):
        
        self.cmds = None
        self.cb = None
        
        rospack = rospkg.RosPack()
        modeldir = rospack.get_path('art_keyword_spotting') + '/models'

        # Create a decoder with certain model
        self.config = Decoder.default_config()
        self.config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
        self.config.set_string('-dict', os.path.join(modeldir, 'en-us/cmudict-en-us.dict'))
        self.config.set_string('-kws',  rospack.get_path('art_keyword_spotting') + '/config/keyphrases.txt') # TODO generate the file from yaml?
        
        self.chunk_size = 4096
        
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=self.chunk_size)
        self.stream.start_stream()
        
        self.decoder = Decoder(self.config)
        self.decoder.start_utt()
        
        self.kw_pub = rospy.Publisher('/art/speech/keywords',  String,  queue_size=10)
        
    def process(self):
        
        while not rospy.is_shutdown():
            
            buf = self.stream.read(self.chunk_size)
            if buf:
                 self.decoder.process_raw(buf, False, False)
            else:
                 break
            if self.decoder.hyp() != None:
                
                word = None
                prob = None
                
                for seg in self.decoder.seg():
                    print (seg.word, seg.prob) 
                    if seg.prob > prob:
                        prob = seg.prob
                        word = seg.word
                print
                
                if word is not None: self.kw_pub.publish(word)
                
                self.decoder.end_utt()
                self.decoder.start_utt()
        
def main(args):
    
    rospy.init_node('simple_gui', anonymous=True)

    speech = art_speech()
    speech.process()
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
