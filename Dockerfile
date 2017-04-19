FROM robotest

CMD ["/bin/bash"]

# install rostrace
ADD rostrace /rostrace
#RUN mkdir -p /tmp/rostrace
#ADD rostrace /tmp/rostrace/rostrace
#ADD setup.py /tmp/rostrace/setup.py
#RUN cd /tmp/rostrace && \
#    sudo ./setup.py install && \
#    cd / && \
#    rm -rf /tmp/*
