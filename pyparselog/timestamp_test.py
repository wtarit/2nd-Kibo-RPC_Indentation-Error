from datetime import datetime

fmt = '%Y%m%d %H%M%S%f'
tstamp1 = datetime.strptime('19700101 000202768', fmt)
tstamp2 = datetime.strptime('19700101 000323320', fmt)

td = tstamp2 - tstamp1
# td_mins = int(round(td.total_seconds() / 60))

print(str(td).split(":",1)[1])
