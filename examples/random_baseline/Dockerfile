FROM python:3.11.18-slim@sha256:ffe4b0c76963e64f05484d74dbb3f67fad1effcf3c5c22c2235f0fafc8a9c98f

WORKDIR /gen
COPY entrypoint.py random_gen.py /gen/

ENTRYPOINT ["python3", "/gen/entrypoint.py"]
