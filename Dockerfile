FROM python:3.10
WORKDIR /DataMiner
COPY . /DataMiner/
RUN pip install --no-cache-dir paho-mqtt pymongo
CMD ["python", "data-miner.py"]