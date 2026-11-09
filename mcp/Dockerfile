# The GNC MCP server in a container, so the host needs only Docker (Linux has
# no `py` launcher, and Arch/Debian refuse a system-wide pip install).
#   docker build -t gnc_moon_simulator/mcp:dev mcp
# .mcp.json runs it on stdio; the tests: docker run --rm --entrypoint python
#   gnc_moon_simulator/mcp:dev -m pytest -q -p no:cacheprovider /mcp
FROM python:3.12-slim
COPY requirements.txt /mcp/requirements.txt
RUN pip install --no-cache-dir --disable-pip-version-check -r /mcp/requirements.txt
COPY gnc_mcp.py test_gnc_mcp.py /mcp/
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
ENTRYPOINT ["python", "/mcp/gnc_mcp.py"]
