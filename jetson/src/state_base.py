class StateBase:
    def run(self, context):
        raise NotImplementedError("Each state must implement the 'run' method.")