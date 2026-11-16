"""Isolated proposal worker. Parent owns deadline, cancellation and archive."""
import sys
from .catalog import Catalog
from .contracts import ContractError, loads, now
from .project import atomic
from .providers import reference, live, live_config
from .electrical import review, report


def main():
    request=loads(sys.stdin.buffer.read(1_000_001))
    project=request['project'];catalog=Catalog();mode=request['mode'];destination=request['destination']
    previous=[];attempts=[];called=False
    def on_request():
        nonlocal called
        called=True
    for attempt in range(1,3 if mode=='live' else 2):
        try:
            proposal=live(project,catalog,previous,on_request=on_request) if mode=='live' else reference(project,catalog)
            validation=review(project,proposal,catalog)
        except ContractError as exc:
            validation=report([{'code':'provider_error','reason':str(exc)}],[],[])
            proposal=None
        attempts.append({'attempt':attempt,'at':now(),'proposal':proposal,'validation':validation})
        atomic(destination,{'mode':mode,'model':live_config()['MODEL'] if mode=='live' else None,'reference_version':'pico2_imu_sg90_v1','live_model_called':called,'attempts':attempts})
        if validation['logical_valid']: break
        previous=validation['errors']
    return 0

if __name__=='__main__':
    sys.exit(main())
