"""Teaching transition protocol; ordinary motion remains exclusively in MoveIt."""
import http.client
import xmlrpc.client


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=4)


def robot_mode(ip, method, value):
    # Same XML-RPC endpoint and methods as the supplied FAIRINO Python SDK.
    # No second SDK/20004 feedback connection is opened.
    with xmlrpc.client.ServerProxy('http://' + ip + ':20003', transport=TimeoutTransport()) as rpc:
        code = getattr(rpc, method)(value)
    if code != 0:
        raise RuntimeError(f'{ip}: {method}({value}) failed: {code}')


class TeachingMode:
    def __init__(self, switch, command):
        self.switch, self.command = switch, command
        self.state = 'motion'

    def enter(self):
        self.state = 'transition'
        try:
            # Stop BOTH streams before enabling either arm's drag mode.
            for side in ('left', 'right'):
                self.switch(side, False)
            for side in ('left', 'right'):
                self.command(side, 'Mode', 1)
                self.command(side, 'DragTeachSwitch', 1)
            self.state = 'teaching'
        except Exception:
            self.state = 'fault'
            # Never restart streaming with an arm possibly still in drag mode.
            raise

    def leave(self):
        self.state = 'transition'
        try:
            # Also serves as explicit recovery after a partial transition.
            for side in ('left', 'right'):
                self.switch(side, False)
            for side in ('left', 'right'):
                self.command(side, 'DragTeachSwitch', 0)
                self.command(side, 'Mode', 0)
            for side in ('left', 'right'):
                self.switch(side, True)
            self.state = 'motion'
        except Exception:
            self.state = 'fault'
            for side in ('left', 'right'):
                try:
                    self.switch(side, False)
                except Exception:
                    pass
            raise
