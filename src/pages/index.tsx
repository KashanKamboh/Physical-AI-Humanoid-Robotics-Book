import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="hero__container">
          <div className="hero__logo">
            <img
              src={require('@site/static/img/robotics-logo.png').default}
              alt="Humanoid Robotics Logo"
              className={styles.heroLogo}
            />
          </div>
          <Heading as="h1" className="hero__title">
            <Translate id="title" description="The title of the homepage">
              Physical AI & Humanoid Robotics
            </Translate>
          </Heading>
          <p className="hero__subtitle">
            <Translate id="tagline" description="The tagline of the homepage">
              Complete Educational Resource for Advanced Robotics Systems
            </Translate>
          </p>
          <p className="hero__description">
            <Translate
              id="description"
              description="The description of the homepage"
            >
              A comprehensive textbook covering the complete pipeline from robotic fundamentals to advanced Vision-Language-Action systems
            </Translate>
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              <Translate
                id="button.primary"
                description="The primary button text"
              >
                Start Learning - 15 Modules ⏱️
              </Translate>
            </Link>
            <Link
              className="button button--primary button--lg margin-left--md"
              to="/docs/module-1/intro">
              <Translate
                id="button.secondary"
                description="The secondary button text"
              >
                Begin with Module 1
              </Translate>
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A comprehensive textbook on Physical AI and Humanoid Robotics - from ROS 2 fundamentals to advanced Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
