class PayloadCodec():

    VERSION: int = 1

    def encode_command(cmd, seq) -> bytes:
        pass

    def decode_status(payload) -> HMIStatus:
        pass