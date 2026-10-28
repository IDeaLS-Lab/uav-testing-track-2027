# The SVM generator. The model is trained OUTSIDE this image, by train.py, and copied in.
#
# The container never trains and never reads the training data: at evaluation time it has no
# network and no data mount. It loads model.json, samples scenarios, scores them, and writes
# the best ones. The image must be rebuilt after every retrain.
FROM python:3.11.18-slim@sha256:ffe4b0c76963e64f05484d74dbb3f67fad1effcf3c5c22c2235f0fafc8a9c98f

RUN useradd -m -u 10001 entrant
WORKDIR /app

# model.json travels with the image. It is plain JSON, so no scikit-learn is needed here and
# the image need not match the version used for training.
COPY generate.py /app/generate.py
COPY model.json  /app/model.json

USER entrant
ENTRYPOINT ["python", "/app/generate.py"]
