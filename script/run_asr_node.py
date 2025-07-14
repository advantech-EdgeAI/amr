import argparse
import asr_node_lite

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-path", required=False, default="./script/config.yaml")
    args = parser.parse_args()

    asr_node_lite.main()