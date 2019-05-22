import rockBlock

from rockBlock import rockBlockProtocol
class Iridium (rockBlockProtocol):
    def write(self,msg):
        rb = rockBlock.rockBlock("/dev/ttyS0", self)
        print("noob!")
        rb.sendMessage(msg)
        print("nooooooob!")
        rb.close()

    #def rockBlockTxStarted(self):
       # print "Redundant Message"

    def rockBlockTxFailed(self):
        print "rockBlockTxFailed"

    def rockBlockTxSuccess(self,momsn):
        print "rockBlockTxSuccess " + str(momsn)

tmp = "Iridiu..Yes..."
Iridium().write(tmp)

Iridium().write("Second message")

